const express = require("express");
const router = express.Router();
const rosBridgeService = require("../services/rosBridgeService");

// POST /api/robot/empty-bins — calls /bins/reset ROS2 service
router.post("/empty-bins", async (req, res) => {
  try {
    const result = await rosBridgeService.callEmptyBins();
    res.json({ success: true, result });
  } catch (err) {
    console.error("empty-bins error:", err.message);
    res.status(500).json({ error: "Failed to call empty bins service: " + err.message });
  }
});

// POST /api/robot/shutdown — calls /robot/shutdown ROS2 service
router.post("/shutdown", async (req, res) => {
  try {
    const result = await rosBridgeService.callShutdown();
    res.json({ success: true, result });
  } catch (err) {
    console.error("shutdown error:", err.message);
    res.status(500).json({ error: "Failed to call shutdown service: " + err.message });
  }
});

// POST /api/robot/teleop — publish geometry_msgs/Twist to /cmd_vel
router.post("/teleop", (req, res) => {
  try {
    const { linear, angular } = req.body;
    if (typeof linear !== "number" || typeof angular !== "number") {
      return res.status(400).json({ error: "linear and angular must be numbers" });
    }
    rosBridgeService.publishCmdVel(linear, angular);
    res.json({ success: true });
  } catch (err) {
    console.error("teleop error:", err.message);
    res.status(500).json({ error: err.message });
  }
});

// POST /api/robot/stop — publish zero velocity to /cmd_vel
router.post("/stop", (req, res) => {
  try {
    rosBridgeService.publishCmdVel(0, 0);
    res.json({ success: true });
  } catch (err) {
    console.error("stop error:", err.message);
    res.status(500).json({ error: err.message });
  }
});

module.exports = router;
