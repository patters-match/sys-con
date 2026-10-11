#include "SwitchVirtualGamepadHandler.h"
#include "SwitchLogger.h"
#include <chrono>
#include <cassert>

namespace
{
    // Half the stick's travel, so a resting stick never turns L+R into a Joy-Con chord.
    constexpr s32 ChordStickThreshold = JOYSTICK_MAX / 2;

    // ZL+ZR counts too: pads that put their shoulder pair on ZL/ZR (fighting and NSO layouts) keep it.
    bool HoldsReattachChord(u64 buttons)
    {
        return ((buttons & HidNpadButton_L) && (buttons & HidNpadButton_R)) ||
               ((buttons & HidNpadButton_ZL) && (buttons & HidNpadButton_ZR));
    }

    /*
        Holding Left or Right with the chord picks a sideways Joy-Con of that side instead of the
        configured type, for games that only take single Joy-Cons: a virtual Joy-Con can never
        send SL+SR (HDLS masks bits 20-27), so this is the only way to get one.
    */
    bool ChordJoyCon(u64 buttons, const HidAnalogStickState &stick, ControllerType *type_out)
    {
        if ((buttons & HidNpadButton_Left) || stick.x < -ChordStickThreshold)
            *type_out = ControllerType_JoyLeft;
        else if ((buttons & HidNpadButton_Right) || stick.x > ChordStickThreshold)
            *type_out = ControllerType_JoyRight;
        else
            return false;
        return true;
    }

    s32 DpadAxis(u64 buttons, u64 negative, u64 positive, s32 stick)
    {
        if (buttons & negative)
            return JOYSTICK_MIN;
        if (buttons & positive)
            return JOYSTICK_MAX;
        return stick;
    }

    /*
        Games read a sideways Joy-Con turned a quarter: measured in Virtua Racing, a left one's
        Right button is the top face button, Up the right one, and stick up steers left. The
        pad's face buttons are moved to the Joy-Con button in the same place once turned, and
        its stick and D-pad - a lone Joy-Con has one stick and no D-pad - turned back, so the
        pad plays as it is held. Left/Down for the left Joy-Con's bottom/left are inferred.
    */
    void ToSidewaysJoyCon(ControllerType side, u64 *buttons, HidAnalogStickState *stick_l, HidAnalogStickState *stick_r)
    {
        constexpr struct
        {
            u64 pad;
            u64 left;
            u64 right;
        } FaceButtons[] = {
            {HidNpadButton_X, HidNpadButton_Right, HidNpadButton_Y},
            {HidNpadButton_A, HidNpadButton_Up, HidNpadButton_X},
            {HidNpadButton_B, HidNpadButton_Left, HidNpadButton_A},
            {HidNpadButton_Y, HidNpadButton_Down, HidNpadButton_B},
        };

        const bool left = side == ControllerType_JoyLeft;
        const u64 pad = *buttons;

        u64 joycon = pad & (HiddbgNpadButton_Home | HiddbgNpadButton_Capture);
        for (const auto &entry : FaceButtons)
        {
            if (pad & entry.pad)
                joycon |= left ? entry.left : entry.right;
        }

        if (pad & (HidNpadButton_Plus | HidNpadButton_Minus))
            joycon |= left ? HidNpadButton_Minus : HidNpadButton_Plus;
        if (pad & (HidNpadButton_StickL | HidNpadButton_StickR))
            joycon |= left ? HidNpadButton_StickL : HidNpadButton_StickR;
        joycon |= pad & (left ? (HidNpadButton_L | HidNpadButton_ZL) : (HidNpadButton_R | HidNpadButton_ZR));

        const s32 x = DpadAxis(pad, HidNpadButton_Left, HidNpadButton_Right, stick_l->x);
        const s32 y = DpadAxis(pad, HidNpadButton_Down, HidNpadButton_Up, stick_l->y);

        *buttons = joycon;
        *stick_l = {};
        *stick_r = {};
        if (left)
            *stick_l = {y, -x};
        else
            *stick_r = {-y, x};
    }
} // namespace

SwitchVirtualGamepadHandler::SwitchVirtualGamepadHandler(std::unique_ptr<IController> &&controller, int32_t polling_timeout_ms, int8_t thread_priority)
    : m_controller(std::move(controller)),
      m_polling_thread_priority(thread_priority),
      m_polling_timeout_ms(polling_timeout_ms)
{
}

SwitchVirtualGamepadHandler::~SwitchVirtualGamepadHandler()
{
    // Do NOT call Exit() here: calling virtuals in destructor is unsafe
    // User must call Exit() manually before destruction in the derived class.
    assert(!m_ThreadIsRunning);
}

Result SwitchVirtualGamepadHandler::Initialize()
{
    syscon::logger::LogDebug("SwitchVirtualGamepadHandler[%04x-%04x] Initializing ...", m_controller->GetDevice()->GetVendor(), m_controller->GetDevice()->GetProduct());

    Result rc = m_controller->Initialize();
    if (R_FAILED(rc))
        return rc;

    return 0;
}

void SwitchVirtualGamepadHandler::Exit()
{
    // Capture these before m_controller->Exit() tears down the underlying device - querying
    // them afterwards crashed (null deref) during power state (sleep/shutdown) transitions.
    uint16_t vendor = m_controller->GetDevice()->GetVendor();
    uint16_t product = m_controller->GetDevice()->GetProduct();

    syscon::logger::LogDebug("SwitchVirtualGamepadHandler[%04x-%04x] Exiting ...", vendor, product);

    ExitThread();

    m_controller->Exit();

    for (int i = 0; i < m_controller->GetInputCount(); i++)
        DetachController(i);

    syscon::logger::LogInfo("SwitchVirtualGamepadHandler[%04x-%04x] Uninitialized !", vendor, product);
}

void SwitchVirtualGamepadHandler::OnRun()
{
    Result rc;
    ::syscon::logger::LogDebug("SwitchVirtualGamepadHandler InputThread running ...");

    /*
     Read timeout depends on the polling frequency and number of interfaces
      - On XBOX360 controllers, we have 4 interfaces, so the read timeout is divided by 4 to make sure we read all interfaces in time.
      - On most of other controllers, we have 1 interface, so the read timeout is equal to the polling frequency but
        it don't have a big impact on the performance - It's even better to set a bigger timeout to avoid the thread to be too busy.
    */
    uint32_t polling_timeout_us = (m_polling_timeout_ms * 1000) / m_controller->GetDevice()->GetInterfaces().size();

    do
    {
        auto startTimer = std::chrono::steady_clock::now();

        rc = UpdateInput(polling_timeout_us);
        (void)UpdateOutput();

        s64 execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - startTimer).count();

        if ((rc != CONTROLLER_STATUS_TIMEOUT) && (execution_time_us > 30000)) // 30ms
            ::syscon::logger::LogWarning("SwitchVirtualGamepadHandler UpdateInputOutput took: %d ms !", execution_time_us / 1000);

        if (R_FAILED(rc) && rc != CONTROLLER_STATUS_TIMEOUT && rc != CONTROLLER_STATUS_NOTHING_TODO)
        {
            /*
            This case is a "normal case" and happen when the controller is disconnected
            Sleep for 100ms second before retrying, provide time to other threads to detect the controller disconnection
            Otherwise, the thread will be too busy and will not let the other threads to run and the nintendo switch will freeze
            */

            //::syscon::logger::LogError("SwitchVirtualGamepadHandler UpdateInputOutput failed with error: 0x%08X !", rc);
            svcSleepThread(100000);
        }

    } while (m_ThreadIsRunning);

    ::syscon::logger::LogDebug("SwitchVirtualGamepadHandler InputThread stopped !");
}

void SwitchVirtualGamepadHandlerThreadFunc(void *handler)
{
    static_cast<SwitchVirtualGamepadHandler *>(handler)->OnRun();
}

Result SwitchVirtualGamepadHandler::InitThread()
{
    m_ThreadIsRunning = true;
    Result rc = threadCreate(&m_Thread, &SwitchVirtualGamepadHandlerThreadFunc, this, thread_stack, sizeof(thread_stack), m_polling_thread_priority, 3 /* On CPU 3 responsible for input */);
    if (R_FAILED(rc))
        return rc;

    rc = threadStart(&m_Thread);
    if (R_FAILED(rc))
        return rc;

    return 0;
}

void SwitchVirtualGamepadHandler::ExitThread()
{
    m_ThreadIsRunning = false;
    svcCancelSynchronization(m_Thread.handle);
    threadWaitForExit(&m_Thread);
    threadClose(&m_Thread);
}

Result SwitchVirtualGamepadHandler::UpdateInput(uint32_t timeout_us)
{
    uint16_t input_idx = 0;
    NormalizedButtonData buttonData = {0};
    u64 buttons = 0;
    HidAnalogStickState analog_stick_l;
    HidAnalogStickState analog_stick_r;

    Result read_rc = m_controller->ReadInput(&buttonData, &input_idx, timeout_us);

    /*
        Note: We must not return here if readInput fail, because it might have change the ControllerConnected state.
        So, we must check if the controller is connected and detach it if it's not.
        This case happen with wireless Xbox 360 controllers
    */

    if (m_controllerData[input_idx].m_is_connected != m_controller->IsControllerConnected(input_idx)) // State changed ?
    {
        syscon::logger::LogDebug("SwitchVirtualGamepadHandler[%04x-%04x] Controller connection state changed on idx: %d !", m_controller->GetDevice()->GetVendor(), m_controller->GetDevice()->GetProduct(), input_idx);

        m_controllerData[input_idx].m_is_connected = m_controller->IsControllerConnected(input_idx);
        if (m_controllerData[input_idx].m_is_connected)
        {
            // If state change to connected, we need to re-attach the controller ASAP
            m_controllerData[input_idx].m_reattach_controller = true;
            m_controllerData[input_idx].m_attach_type = m_controller->GetConfig().controllerType;
        }
        else
        {
            syscon::logger::LogDebug("SwitchVirtualGamepadHandler[%04x-%04x] Detaching controller on idx: %d !", m_controller->GetDevice()->GetVendor(), m_controller->GetDevice()->GetProduct(), input_idx);
            DetachController(input_idx);
        }
    }

    if (m_controllerData[input_idx].m_is_connected == false)
        return read_rc; // No need to update the controller state if it's not connected

    if (R_FAILED(read_rc))
        return read_rc;

    auto startTimer = std::chrono::steady_clock::now();

    if (buttonData.buttons[ControllerButton::X])
        buttons |= HidNpadButton_X;
    if (buttonData.buttons[ControllerButton::A])
        buttons |= HidNpadButton_A;
    if (buttonData.buttons[ControllerButton::B])
        buttons |= HidNpadButton_B;
    if (buttonData.buttons[ControllerButton::Y])
        buttons |= HidNpadButton_Y;
    if (buttonData.buttons[ControllerButton::LSTICK_CLICK])
        buttons |= HidNpadButton_StickL;
    if (buttonData.buttons[ControllerButton::RSTICK_CLICK])
        buttons |= HidNpadButton_StickR;
    if (buttonData.buttons[ControllerButton::L])
        buttons |= HidNpadButton_L;
    if (buttonData.buttons[ControllerButton::R])
        buttons |= HidNpadButton_R;
    if (buttonData.buttons[ControllerButton::ZL])
        buttons |= HidNpadButton_ZL;
    if (buttonData.buttons[ControllerButton::ZR])
        buttons |= HidNpadButton_ZR;
    if (buttonData.buttons[ControllerButton::MINUS])
        buttons |= HidNpadButton_Minus;
    if (buttonData.buttons[ControllerButton::PLUS])
        buttons |= HidNpadButton_Plus;
    if (buttonData.buttons[ControllerButton::DPAD_UP])
        buttons |= HidNpadButton_Up;
    if (buttonData.buttons[ControllerButton::DPAD_RIGHT])
        buttons |= HidNpadButton_Right;
    if (buttonData.buttons[ControllerButton::DPAD_DOWN])
        buttons |= HidNpadButton_Down;
    if (buttonData.buttons[ControllerButton::DPAD_LEFT])
        buttons |= HidNpadButton_Left;
    if (buttonData.buttons[ControllerButton::CAPTURE])
        buttons |= HiddbgNpadButton_Capture;
    if (buttonData.buttons[ControllerButton::HOME])
        buttons |= HiddbgNpadButton_Home;

    ConvertAxisToSwitchAxis(buttonData.sticks[0].axis_x, buttonData.sticks[0].axis_y, &analog_stick_l.x, &analog_stick_l.y);
    ConvertAxisToSwitchAxis(buttonData.sticks[1].axis_x, buttonData.sticks[1].axis_y, &analog_stick_r.x, &analog_stick_r.y);

    /*
        The type is settled on the first report with the chord held, so the direction has to be
        held before L+R or ZL+ZR is pressed.
    */
    SwitchVirtualGamepadHandlerData &data = m_controllerData[input_idx];
    if (!IsControllerAttached(input_idx) && !data.m_reattach_controller && HoldsReattachChord(buttons))
    {
        data.m_attach_type = m_controller->GetConfig().controllerType;
        ChordJoyCon(buttons, analog_stick_l, &data.m_attach_type);
        data.m_reattach_controller = true;
    }

    if (m_controllerData[input_idx].m_reattach_controller)
    {
        syscon::logger::LogInfo("SwitchVirtualGamepadHandler[%04x-%04x] Re-attaching controller on idx: %d as type %d !", m_controller->GetDevice()->GetVendor(), m_controller->GetDevice()->GetProduct(), input_idx, m_controllerData[input_idx].m_attach_type);
        AttachController(input_idx);
        m_controllerData[input_idx].m_reattach_controller = false;
    }

    // We get the button inputs from the input packet and update the state of our controller
    syscon::logger::LogDebug("SwitchVirtualGamepadHandler[%04x-%04x] Updating controller state on idx: %d !", m_controller->GetDevice()->GetVendor(), m_controller->GetDevice()->GetProduct(), input_idx);
    if (IsJoyCon(m_controllerData[input_idx].m_attach_type))
        ToSidewaysJoyCon(m_controllerData[input_idx].m_attach_type, &buttons, &analog_stick_l, &analog_stick_r);

    Result res = UpdateControllerState(buttons, analog_stick_l, analog_stick_r, input_idx);

    s64 execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - startTimer).count();
    syscon::logger::LogPerf("SwitchVirtualGamepadHandler[%04x-%04x] UpdateInput took: %d us for idx: %d !", m_controller->GetDevice()->GetVendor(), m_controller->GetDevice()->GetProduct(), execution_time_us, input_idx);

    return res;
}

Result SwitchVirtualGamepadHandler::UpdateOutput()
{
    // Vibrations are not supported with HDL
    return 0;
}

void SwitchVirtualGamepadHandler::ConvertAxisToSwitchAxis(float x, float y, int32_t *x_out, int32_t *y_out)
{
    float floatRange = 2.0f;
    float newRange = (JOYSTICK_MAX - JOYSTICK_MIN);

    *x_out = (((x + 1.0f) * newRange) / floatRange) + JOYSTICK_MIN;
    *y_out = -((((y + 1.0f) * newRange) / floatRange) + JOYSTICK_MIN);
}

void SwitchVirtualGamepadHandler::BuildHdlsDeviceInfo(HiddbgHdlsDeviceInfo *deviceInfo, uint16_t input_idx)
{
    memset(deviceInfo, 0, sizeof(*deviceInfo));

    const ControllerType type = m_controllerData[input_idx].m_attach_type;
    deviceInfo->deviceType = ControllerTypeToDeviceType(type);
    // Rail would merge the Joy-Con into an existing controller, and Joy-Cons have no USB mode.
    deviceInfo->npadInterfaceType = IsJoyCon(type) ? HidNpadInterfaceType_Bluetooth : HidNpadInterfaceType_USB;

    // The grip colors are for Pro-Controller on [9.0.0+].
    deviceInfo->singleColorBody = __builtin_bswap32(m_controller->GetConfig().bodyColor.rgbaValue);
    deviceInfo->singleColorButtons = __builtin_bswap32(m_controller->GetConfig().buttonsColor.rgbaValue);
    deviceInfo->colorLeftGrip = __builtin_bswap32(m_controller->GetConfig().leftGripColor.rgbaValue);
    deviceInfo->colorRightGrip = __builtin_bswap32(m_controller->GetConfig().rightGripColor.rgbaValue);
}

u8 SwitchVirtualGamepadHandler::ControllerTypeToDeviceType(ControllerType type)
{
    if (type == ControllerType_ProWithBattery)
        return HidDeviceType_FullKey3;
    else if (type == ControllerType_Tarragon)
        return HidDeviceType_FullKey6;
    else if (type == ControllerType_Snes)
        return HidDeviceType_Lucia;
    else if (type == ControllerType_PokeballPlus)
        return HidDeviceType_Palma;
    else if (type == ControllerType_Gamecube)
        return HidDeviceType_FullKey13;
    else if (type == ControllerType_Pro)
        return HidDeviceType_FullKey15;
    else if (type == ControllerType_3rdPartyPro)
        return HidDeviceType_System19;
    else if (type == ControllerType_N64)
        return HidDeviceType_Lagon;
    else if (type == ControllerType_Sega)
        return HidDeviceType_Lager;
    else if (type == ControllerType_Nes)
        return HidDeviceType_LarkNesLeft;
    else if (type == ControllerType_Famicom)
        return HidDeviceType_LarkHvcLeft;
    else if (type == ControllerType_JoyLeft)
        return HidDeviceType_JoyLeft2;
    else if (type == ControllerType_JoyRight)
        return HidDeviceType_JoyRight1;

    return HidDeviceType_FullKey15;
}