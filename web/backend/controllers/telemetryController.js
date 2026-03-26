const mongoose = require("mongoose");
const RobotTelemetry = require("../models/RobotTelemetry");
const rosBridgeService = require("../services/rosBridgeService");

// GET /api/telemetry/:robotId/latest
const getLatestTelemetry = async (req, res) => {
  try {
    const { robotId } = req.params;
    if (!mongoose.Types.ObjectId.isValid(robotId)) {
      return res.status(400).json({ error: "Invalid robot ID" });
    }

    // First try live data from ROS bridge
    const live = rosBridgeService.getLatestData(robotId);
    if (live) {
      return res.json({ source: "live", data: live });
    }

    // Fall back to most recent DB record
    const telemetry = await RobotTelemetry.findOne({ robotId }).sort({
      timestamp: -1,
    });

    if (!telemetry) {
      return res.status(404).json({ error: "No telemetry found" });
    }

    res.json({ source: "db", data: telemetry });
  } catch (err) {
    res.status(500).json({ error: "Failed to fetch telemetry" });
  }
};

// GET /api/telemetry/:robotId/history?from=&to=&limit=
const getTelemetryHistory = async (req, res) => {
  try {
    const { robotId } = req.params;
    if (!mongoose.Types.ObjectId.isValid(robotId)) {
      return res.status(400).json({ error: "Invalid robot ID" });
    }

    const { from, to, limit } = req.query;
    const query = { robotId };

    if (from || to) {
      query.timestamp = {};
      if (from) {
        const fromDate = new Date(from);
        if (isNaN(fromDate.getTime())) {
          return res.status(400).json({ error: "Invalid 'from' date" });
        }
        query.timestamp.$gte = fromDate;
      }
      if (to) {
        const toDate = new Date(to);
        if (isNaN(toDate.getTime())) {
          return res.status(400).json({ error: "Invalid 'to' date" });
        }
        query.timestamp.$lte = toDate;
      }
    }

    const parsedLimit = Math.min(
      Math.max(parseInt(limit) || 100, 1),
      1000
    );

    const telemetry = await RobotTelemetry.find(query)
      .sort({ timestamp: -1 })
      .limit(parsedLimit);

    res.json(telemetry);
  } catch (err) {
    res.status(500).json({ error: "Failed to fetch telemetry history" });
  }
};

// GET /api/telemetry/all/latest
const getAllLatestTelemetry = async (req, res) => {
  try {
    const allLive = rosBridgeService.getAllLatestData();
    res.json(allLive);
  } catch (err) {
    res.status(500).json({ error: "Failed to fetch telemetry" });
  }
};

module.exports = {
  getLatestTelemetry,
  getTelemetryHistory,
  getAllLatestTelemetry,
};
