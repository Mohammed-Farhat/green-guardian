const mongoose = require("mongoose");
const Robot = require("../models/Robot");
const rosBridgeService = require("../services/rosBridgeService");

function isValidId(id) {
  return mongoose.Types.ObjectId.isValid(id);
}

// GET /api/robots
const getRobots = async (req, res) => {
  try {
    const robots = await Robot.find().sort({ name: 1 });
    res.json(robots);
  } catch (err) {
    res.status(500).json({ error: "Failed to fetch robots" });
  }
};

// GET /api/robots/:id
const getRobotById = async (req, res) => {
  try {
    if (!isValidId(req.params.id)) {
      return res.status(400).json({ error: "Invalid robot ID" });
    }
    const robot = await Robot.findById(req.params.id);
    if (!robot) return res.status(404).json({ error: "Robot not found" });
    res.json(robot);
  } catch (err) {
    res.status(500).json({ error: "Failed to fetch robot" });
  }
};

// POST /api/robots
const createRobot = async (req, res) => {
  try {
    const { name, status, rosTopics } = req.body;
    if (!name) {
      return res.status(400).json({ error: "Robot name is required" });
    }
    const robot = await Robot.create({ name, status, rosTopics });
    res.status(201).json(robot);
  } catch (err) {
    if (err.code === 11000) {
      return res.status(400).json({ error: "Robot name already exists" });
    }
    res.status(400).json({ error: "Failed to create robot" });
  }
};

// PUT /api/robots/:id
const updateRobot = async (req, res) => {
  try {
    if (!isValidId(req.params.id)) {
      return res.status(400).json({ error: "Invalid robot ID" });
    }
    const { name, status, rosTopics } = req.body;
    const robot = await Robot.findByIdAndUpdate(
      req.params.id,
      { name, status, rosTopics },
      { new: true, runValidators: true }
    );
    if (!robot) return res.status(404).json({ error: "Robot not found" });

    if (status === "online") {
      rosBridgeService.subscribeToRobot(robot);
    } else if (status === "offline") {
      rosBridgeService.unsubscribeRobot(robot._id.toString());
    }

    res.json(robot);
  } catch (err) {
    res.status(400).json({ error: "Failed to update robot" });
  }
};

// DELETE /api/robots/:id
const deleteRobot = async (req, res) => {
  try {
    if (!isValidId(req.params.id)) {
      return res.status(400).json({ error: "Invalid robot ID" });
    }
    const robot = await Robot.findByIdAndDelete(req.params.id);
    if (!robot) return res.status(404).json({ error: "Robot not found" });
    rosBridgeService.unsubscribeRobot(robot._id.toString());
    res.json({ message: "Robot deleted" });
  } catch (err) {
    res.status(500).json({ error: "Failed to delete robot" });
  }
};

module.exports = {
  getRobots,
  getRobotById,
  createRobot,
  updateRobot,
  deleteRobot,
};
