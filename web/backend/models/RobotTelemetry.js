const mongoose = require("mongoose");

const robotTelemetrySchema = new mongoose.Schema(
  {
    robotId: {
      type: mongoose.Schema.Types.ObjectId,
      ref: "Robot",
      required: true,
      index: true,
    },
    speed: {
      type: Number,
      default: 0,
    },
    temperature: {
      type: Number,
      default: 0,
    },
    humidity: {
      type: Number,
      default: 0,
    },
    battery: {
      type: Number,
      default: 0,
    },
    binOrganic: {
      type: Number,
      default: 0,
    },
    binNonOrganic: {
      type: Number,
      default: 0,
    },
    cpuTemp: {
      type: Number,
      default: 0,
    },
    fanSpeed: {
      type: Number,
      default: 0,
    },
    gps: {
      lat: { type: Number, default: 0 },
      lng: { type: Number, default: 0 },
    },
    timestamp: {
      type: Date,
      default: Date.now,
      index: true,
    },
  },
  { timestamps: true }
);

// TTL index: auto-delete telemetry older than 7 days
robotTelemetrySchema.index({ timestamp: 1 }, { expireAfterSeconds: 604800 });

module.exports = mongoose.model("RobotTelemetry", robotTelemetrySchema);
