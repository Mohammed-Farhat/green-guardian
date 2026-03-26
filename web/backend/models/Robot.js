const mongoose = require("mongoose");

const robotSchema = new mongoose.Schema(
  {
    name: {
      type: String,
      required: true,
      unique: true,
    },
    status: {
      type: String,
      enum: ["online", "offline", "maintenance"],
      default: "offline",
    },
    lastSeen: {
      type: Date,
      default: Date.now,
    },
    rosTopics: {
      speed: { type: String, default: "/odom" },
      temperature: { type: String, default: "/sensor/temperature" },
      humidity: { type: String, default: "/sensor/humidity" },
      gps: { type: String, default: "/gps/fix" },
      battery: { type: String, default: "/battery_state" },
      binStatus: { type: String, default: "/bin_status" },
      cpuTemp: { type: String, default: "/cpu_temp" },
      fanSpeed: { type: String, default: "/fan_speed" },
    },
  },
  { timestamps: true }
);

module.exports = mongoose.model("Robot", robotSchema);
