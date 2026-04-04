const express = require("express");
const router = express.Router();
const rosBridgeService = require("../services/rosBridgeService");

// POST /api/robot/empty-bins — calls /bins/reset ROS2 service
router.post("/empty-bins", async (req, res) => {
  try {
    const result = await rosBridgeService.callEmptyBins();
    res.json({ success: true, result });
  } catch (err) {
    res.status(500).json({ error: "Failed to call empty bins service: " + err.message });
  }
});

// POST /api/robot/shutdown — calls /robot/shutdown ROS2 service
router.post("/shutdown", async (req, res) => {
  try {
    const result = await rosBridgeService.callShutdown();
    res.json({ success: true, result });
  } catch (err) {
    res.status(500).json({ error: "Failed to call shutdown service: " + err.message });
  }
});

module.exports = router;
