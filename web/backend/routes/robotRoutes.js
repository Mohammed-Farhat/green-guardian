const express = require("express");
const router = express.Router();
const auth = require("../middleware/auth");
const {
  getRobots,
  getRobotById,
  createRobot,
  updateRobot,
  deleteRobot,
} = require("../controllers/robotController");

router.get("/", auth, getRobots);
router.get("/:id", auth, getRobotById);
router.post("/", auth, createRobot);
router.put("/:id", auth, updateRobot);
router.delete("/:id", auth, deleteRobot);

module.exports = router;
