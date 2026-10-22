-- Generate an articulated 4-DOF teaching arm and six-block pick/place workspace.
-- Run once from CoppeliaSim's Lua editor in a fresh scene, then save as simulation/scene.ttt.
local sim = require('sim')

local function shape(primitive, size, pos, color, name, mass)
    local h = sim.createPrimitiveShape(primitive, size, 0)
    sim.setObjectAlias(h, name, 0)
    sim.setObjectPosition(h, -1, pos)
    sim.setShapeColor(h, nil, sim.colorcomponent_ambient_diffuse, color)
    if mass then
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 0)
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 1)
        sim.setShapeMass(h, mass)
    else
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
    end
    return h
end

local function parentKeepWorld(child, parent)
    sim.setObjectParent(child, parent, true)
end

local function joint(name, parent, pos, jointType, axisOrientation, limits)
    local j = sim.createJoint(jointType, sim.jointmode_dynamic, 0, {0.06, 0.06})
    sim.setObjectAlias(j, name, 0)
    sim.setObjectPosition(j, -1, pos)
    if axisOrientation then sim.setObjectOrientation(j, -1, axisOrientation) end
    parentKeepWorld(j, parent)
    sim.setJointInterval(j, false, {limits[1], limits[2] - limits[1]})
    pcall(function() sim.setObjectInt32Param(j, sim.jointintparam_motor_enabled, 1) end)
    pcall(function() sim.setObjectInt32Param(j, sim.jointintparam_ctrl_enabled, 1) end)
    pcall(function() sim.setObjectInt32Param(j, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position) end)
    pcall(function() sim.setJointTargetForce(j, 40) end)
    pcall(function() sim.setJointTargetVelocity(j, 0.7) end)
    sim.setJointTargetPosition(j, 0)
    return j
end

-- Work surface top z=0.82m; tray floor top z=0.8475m.
shape(sim.primitiveshape_plane, {3, 3, 0.02}, {0, 0, 0}, {0.20, 0.22, 0.25}, 'ground')
shape(sim.primitiveshape_cuboid, {1.4, 0.9, 0.08}, {0.45, 0, 0.78}, {0.32, 0.35, 0.39}, 'workbench')
shape(sim.primitiveshape_cuboid, {0.36, 0.26, 0.025}, {0.48, -0.23, 0.835}, {0.12, 0.15, 0.18}, 'source_tray')
shape(sim.primitiveshape_cuboid, {0.36, 0.26, 0.025}, {0.48, 0.25, 0.835}, {0.12, 0.15, 0.18}, 'destination_tray')

-- Six non-respondable slot markers on the destination tray.
local slotXs = {0.38, 0.48, 0.58, 0.38, 0.48, 0.58}
local slotYs = {0.20, 0.20, 0.20, 0.30, 0.30, 0.30}
for i = 1, 6 do
    local marker = shape(sim.primitiveshape_cylinder, {0.072, 0.072, 0.002}, {slotXs[i], slotYs[i], 0.849}, {0.40, 0.43, 0.47}, 'destination_slot_' .. i, nil)
    sim.setObjectInt32Param(marker, sim.shapeintparam_respondable, 0)
end

-- Fixed base and vertical riser place the shoulder pivot at (0, 0, 1.03).
local base = shape(sim.primitiveshape_cylinder, {0.18, 0.18, 0.16}, {0, 0, 0.90}, {0.20, 0.35, 0.55}, 'robot_base')
local j0 = joint('joint_base', base, {0, 0, 0.99}, sim.joint_revolute, nil, {-2.8, 2.8})
local riser = shape(sim.primitiveshape_cuboid, {0.08, 0.08, 0.08}, {0, 0, 1.03}, {0.27, 0.40, 0.60}, 'arm_riser', 0.15)
parentKeepWorld(riser, j0)

-- Pitch axes are local Z rotated onto world -Y to match the Python FK convention.
local j1 = joint('joint_shoulder', riser, {0, 0, 1.03}, sim.joint_revolute, {math.pi/2, 0, 0}, {-1.6, 1.6})
local link1 = shape(sim.primitiveshape_cuboid, {0.28, 0.06, 0.06}, {0.14, 0, 1.03}, {0.75, 0.78, 0.82}, 'arm_link_1', 0.30)
parentKeepWorld(link1, j1)
local j2 = joint('joint_elbow', link1, {0.28, 0, 1.03}, sim.joint_revolute, {math.pi/2, 0, 0}, {-2.2, 2.2})
local link2 = shape(sim.primitiveshape_cuboid, {0.30, 0.055, 0.055}, {0.43, 0, 1.03}, {0.78, 0.80, 0.84}, 'arm_link_2', 0.25)
parentKeepWorld(link2, j2)
local j3 = joint('joint_wrist', link2, {0.58, 0, 1.03}, sim.joint_revolute, {math.pi/2, 0, 0}, {-2.0, 2.0})
local link3 = shape(sim.primitiveshape_cuboid, {0.20, 0.05, 0.05}, {0.68, 0, 1.03}, {0.82, 0.83, 0.86}, 'arm_link_3', 0.18)
parentKeepWorld(link3, j3)
local tcp = shape(sim.primitiveshape_cuboid, {0.09, 0.09, 0.06}, {0.78, 0, 1.03}, {0.16, 0.18, 0.20}, 'tool_center_point', 0.12)
parentKeepWorld(tcp, link3)

-- A pair of prismatic joints slides gripper fingers sideways.
local gl = joint('gripper_left_joint', tcp, {0.78, 0, 1.03}, sim.joint_prismatic, {-math.pi/2, 0, 0}, {-0.04, 0.04})
local gr = joint('gripper_right_joint', tcp, {0.78, 0, 1.03}, sim.joint_prismatic, {-math.pi/2, 0, 0}, {-0.04, 0.04})
local fingerL = shape(sim.primitiveshape_cuboid, {0.025, 0.025, 0.075}, {0.78, -0.035, 0.965}, {0.90, 0.62, 0.12}, 'gripper_left_finger', 0.03)
parentKeepWorld(fingerL, gl)
local fingerR = shape(sim.primitiveshape_cuboid, {0.025, 0.025, 0.075}, {0.78, 0.035, 0.965}, {0.90, 0.62, 0.12}, 'gripper_right_finger', 0.03)
parentKeepWorld(fingerR, gr)

local colors = {
    {'red', {0.90, 0.13, 0.13}}, {'blue', {0.12, 0.36, 0.93}},
    {'green', {0.12, 0.72, 0.32}}, {'yellow', {0.98, 0.78, 0.12}},
    {'orange', {0.98, 0.40, 0.10}}, {'purple', {0.60, 0.25, 0.82}}
}
local xs = {0.38, 0.48, 0.58, 0.38, 0.48, 0.58}
local ys = {-0.28, -0.28, -0.28, -0.18, -0.18, -0.18}
for i, c in ipairs(colors) do
    local block = shape(sim.primitiveshape_cuboid, {0.055, 0.055, 0.055}, {xs[i], ys[i], 0.875}, c[2], 'block_' .. c[1], 0.05)
    sim.setObjectInt32Param(block, sim.shapeintparam_respondable, 1)
end

sim.addLog(sim.verbosity_scriptinfos, 'Robotic arm scene created with six destination slots. Save as simulation/scene.ttt before running run_demo.py.')
