import ROSLIB from "roslib";
import { BRAKE_PAGE } from "./actionTrial.js";

// Adaptor ROS untuk actionTrial.js: topik dan layanan op3_manager yang dipakai
// satu percobaan Action Training. onPrepare dari App.jsx (lepas batas kecepatan
// servo + tandai walking_module tidak aktif, sama seperti tombol Init Pose).

const LEG_JOINTS = [
  "r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll", "r_hip_pitch", "l_hip_pitch",
  "r_knee", "l_knee", "r_ank_pitch", "l_ank_pitch", "r_ank_roll", "l_ank_roll",
];
const SERVICE_TIMEOUT_MS = 2000;

export function createActionRobot(ros, { onPrepare } = {}) {
  const page = new ROSLIB.Topic({ ros, name: "/robotis/action/page_num", messageType: "std_msgs/Int32" });
  const module = new ROSLIB.Topic({ ros, name: "/robotis/enable_ctrl_module", messageType: "std_msgs/String" });
  // Diiklankan sekarang: pesan pertama sesudah iklan bisa hilang selagi DDS mencari
  // pelanggannya, padahal pesan pertama di sini bisa jadi rem.
  page.advertise();
  module.advertise();
  const isRunningService = new ROSLIB.Service({
    ros, name: "/robotis/action/is_running", serviceType: "op3_action_module_msgs/srv/IsRunning",
  });
  const jointModuleService = new ROSLIB.Service({
    ros, name: "/robotis/get_present_joint_ctrl_modules", serviceType: "robotis_controller_msgs/srv/GetJointModule",
  });

  const call = (service, request) => new Promise((resolve) => {
    const timer = window.setTimeout(() => resolve(null), SERVICE_TIMEOUT_MS);
    service.callService(
      new ROSLIB.ServiceRequest(request),
      (res) => { window.clearTimeout(timer); resolve(res); },
      () => { window.clearTimeout(timer); resolve(null); }
    );
  });

  return {
    prepare: () => onPrepare?.(),
    useActionModule: () => module.publish(new ROSLIB.Message({ data: "action_module" })),
    legsOwnedBy: async (name) => {
      const res = await call(jointModuleService, { joint_name: LEG_JOINTS });
      if (!res?.module_name?.length) return null;
      return res.module_name.every((m) => m === name);
    },
    playPage: (index) => page.publish(new ROSLIB.Message({ data: index })),
    brake: () => page.publish(new ROSLIB.Message({ data: BRAKE_PAGE })),
    isRunning: async () => {
      const res = await call(isRunningService, {});
      return res ? Boolean(res.is_running) : null;
    },
    close: () => {
      page.unadvertise();
      module.unadvertise();
    },
  };
}
