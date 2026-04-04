const RobotTelemetry = require("../models/RobotTelemetry");
const rosBridgeService = require("../services/rosBridgeService");

// GET /api/telemetry/latest
const getLatestTelemetry = async (req, res) => {
  try {
    // First try live data from ROS bridge
    const live = rosBridgeService.getLatestData();
    if (live) {
      return res.json({ source: "live", data: live });
    }
    // Fall back to most recent DB record
    const telemetry = await RobotTelemetry.findOne().sort({ timestamp: -1 });

    if (!telemetry) {
      return res.status(404).json({ error: "No telemetry found" });
    }

    res.json({ source: "db", data: telemetry });
  } catch (err) {
    res.status(500).json({ error: "Failed to fetch telemetry" });
  }
};

// GET /api/telemetry/history?limit=
const getTelemetryHistory = async (req, res) => {
  try {
    const { limit } = req.query;
    const parsedLimit = Math.min(Math.max(parseInt(limit) || 100, 1), 1000);

    const telemetry = await RobotTelemetry.find()
      .sort({ timestamp: -1 })
      .limit(parsedLimit);

    res.json(telemetry);
  } catch (err) {
    res.status(500).json({ error: "Failed to fetch telemetry history" });
  }
};

module.exports = {
  getLatestTelemetry,
  getTelemetryHistory,
};
