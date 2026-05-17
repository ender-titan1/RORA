#include "control_system.h"
#include "coordinator_common.h"
#include "motion_planner.h"

#define TAG "control_system"

void init_control_system(control_system_t *controller, uint8_t* peer_mac, control_mode_t control_mode, integrator_mode_t integrator_mode, joint_t* joints, size_t joints_len)
{
    controller->mode = control_mode;
    drv8825_set_control_mode(control_mode);

#if CONFIG_DEVICE_ROLE_CORE
    // Configure the satellite
    data_frame_t frame = {
        .command = CMD_CONFIGURE,
    };
    configure_command_payload_t payload = {
        .control_mode = control_mode,
        .integrator_mode = integrator_mode
    };
    memcpy(frame.payload, &payload, sizeof(mp_curve_command_payload_t));
    transmit_frame(peer_mac, &frame);
    if (!await_response())
        ESP_LOGE(TAG, "Failed to recieve ACK on curve command!");
#endif

    // Initialize the device in either core or
    if (control_mode == OFFLINE)
    {
        // TODO: Replace these placeholder values
#if CONFIG_DEVICE_ROLE_CORE
        controller->backend.motion_planner = init_motion_planner(16, 8, joints_len);
        memcpy(controller->backend.motion_planner->satellite_addrs[0], peer_mac, sizeof(uint8_t) * MAC_ADDR_LEN);
        joint_t* joint_arr = controller->backend.motion_planner->core_buffers->joints;
#elif CONFIG_DEVICE_ROLE_SAT
        controller->backend.motion_planner_bufs = malloc(sizeof(controller_specific_buffers_t));
        init_buffers(controller->backend.motion_planner_bufs, 16, 8, joints_len);
        joint_t* joint_arr = controller->backend.motion_planner_bufs->joints;
#endif
        for (size_t i = 0; i < joints_len; i++)
            joint_arr[i] = joints[i];
    }
    else
    {
        controller->backend.realtime_controller = malloc(sizeof(rt_controller_t));
        controller->integrator_mode = integrator_mode;
        init_realtime(controller->backend.realtime_controller);
        for (size_t i = 0; i < joints_len; i++)
            controller->backend.realtime_controller->joints_arr[i] = joints[i];
    }

    // Attach motors 
    for (size_t i = 0; i < joints_len; i++)
    {
        attach_motor(joints[i].motor);

        if (control_mode == OFFLINE)
            disable_motor(joints[i].motor);
    }
    
}

void reset_control_system(control_system_t *controller)
{
    // Detach all motors and delete all malloc'ed resources
    if (controller->mode == OFFLINE)
    {
#if CONFIG_DEVICE_ROLE_CORE
        joint_t* joint_arr = controller->backend.motion_planner->core_buffers->joints;
        size_t len = controller->backend.motion_planner->core_buffers->joint_buf_size;
#elif CONFIG_DEVICE_ROLE_SAT
        joint_t* joint_arr = controller->backend.motion_planner_bufs->joints;
        size_t len = controller->backend.motion_planner->core_buffers->joint_buf_size;
#endif
        for (size_t i = 0; i < len; i++)
            detach_motor(joint_arr[i].motor);

#if CONFIG_DEVICE_ROLE_CORE
        delete_motion_planner(controller->backend.motion_planner);
#elif CONFIG_DEVICE_ROLE_SAT
        delete_local_buffers(controller->backend.motion_planner_bufs);
#endif
    }
}
