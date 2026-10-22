-- Build a non-static articulated primitive arm, trays and colored blocks.
-- Execute once from CoppeliaSim's Lua editor in an empty scene, then save as simulation/scene.ttt.
-- This script requires a modern CoppeliaSim version exposing sim.createJoint / sim.createPrimitiveShape.
local sim = require('sim')

local function shape(primitive, size, pos, color, name, mass)
    local h = sim.createPrimitiveShape(primitive, size, 0)
    sim.setObjectAlias(h, name, 0)
    sim.setObjectPosition(h, -1, pos)
    sim.setShapeColor(h, nil, sim.colorcomponent_ambient_diffuse, color)
    sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
    if mass then
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 0)
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 1)
        sim.setShapeMass(h, mass)
    end
    return h
end

local function revolute(name, parent, pos, axis, limits)
    local j = sim.createJoint(sim.joint revolute or sim.joint_revolute, sim.jointmode_dynamic, 0, {0.08,0.08})
    sim.setObjectAlias(j, name, 0)
    sim.setObjectPosition(j, -1, pos)
    sim.setObjectParent(j, parent, true)
    sim.setJointMode(j, sim.jointmode_kinematic, 0)
    sim.setJointInterval(j, false, {limits[1], limits[2]-limits[1]})
    sim.setObjectInt32Param(j, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position)
    sim.setJointTargetPosition(j, 0)
    return j
end

local function staticShape(primitive, size, pos, color, name)
    local h = shape(primitive, size, pos, color, name, nil)
    return h
end

-- Ground and workbench.
staticShape(sim.primitiveshape_plane, {3,3,0.02}, {0,0,0.0}, {0.20,0.22,0.25}, 'ground')
staticShape(sim.primitiveshape_cuboid, {1.4,0.9,0.08}, {0.45,0,0.78}, {0.32,0.35,0.39}, 'workbench')
-- Simple tray rims.
staticShape(sim.primitiveshape_cuboid, {0.36,0.26,0.025}, {0.48,-0.23,0.835}, {0.12,0.15,0.18}, 'source_tray')
staticShape(sim.primitiveshape_cuboid, {0.36,0.26,0.025}, {0.48,0.25,0.835}, {0.12,0.15,0.18}, 'destination_tray')

-- Four joint chain: base yaw + shoulder + elbow + wrist. Links are dynamic visual shapes,
-- attached to their preceding joint. Joint axes are set by the model's local Z defaults.
local base = staticShape(sim.primitiveshape_cylinder, {0.18,0.18,0.16}, {0,0,0.90}, {0.20,0.35,0.55}, 'robot_base')
local j0 = revolute('joint_base', base, {0,0,1.00}, {0,0,1}, {-2.8,2.8})
local link1 = shape(sim.primitiveshape_cuboid, {0.09,0.09,0.28}, {0.12,0,1.14}, {0.75,0.78,0.82}, 'arm_link_1', 0.3)
sim.setObjectParent(link1, j0, true)
local j1 = revolute('joint_shoulder', j0, {0.12,0,1.28}, {0,1,0}, {-1.6,1.6})
local link2 = shape(sim.primitiveshape_cuboid, {0.07,0.07,0.30}, {0.28,0,1.40}, {0.78,0.80,0.84}, 'arm_link_2', 0.25)
sim.setObjectParent(link2, j1, true)
local j2 = revolute('joint_elbow', j1, {0.28,0,1.55}, {0,1,0}, {-2.2,2.2})
local link3 = shape(sim.primitiveshape_cuboid, {0.06,0.06,0.20}, {0.38,0,1.55}, {0.82,0.83,0.86}, 'arm_link_3', 0.18)
sim.setObjectParent(link3, j2, true)
local j3 = revolute('joint_wrist', j2, {0.38,0,1.55}, {0,1,0}, {-2.0,2.0})
local tcp = staticShape(sim.primitiveshape_cuboid, {0.10,0.10,0.06}, {0.45,0,1.53}, {0.16,0.18,0.20}, 'tool_center_point')
sim.setObjectParent(tcp, j3, true)

-- Parallel finger primitives; visible fingers with their own position target joints.
local gl = revolute('gripper_left_joint', j3, {0.44,-0.035,1.49}, {1,0,0}, {-0.04,0.04})
local gr = revolute('gripper_right_joint', j3, {0.44,0.035,1.49}, {1,0,0}, {-0.04,0.04})
local fingerL = shape(sim.primitiveshape_cuboid, {0.025,0.025,0.10}, {0.46,-0.035,1.43}, {0.9,0.62,0.12}, 'gripper_left_finger', 0.03)
sim.setObjectParent(fingerL, gl, true)
local fingerR = shape(sim.primitiveshape_cuboid, {0.025,0.025,0.10}, {0.46,0.035,1.43}, {0.9,0.62,0.12}, 'gripper_right_finger', 0.03)
sim.setObjectParent(fingerR, gr, true)

local colors = {
    {'red',{0.90,0.13,0.13}}, {'blue',{0.12,0.36,0.93}},
    {'green',{0.12,0.72,0.32}}, {'yellow',{0.98,0.78,0.12}},
    {'orange',{0.98,0.40,0.10}}, {'purple',{0.60,0.25,0.82}}
}
local xs = {0.38,0.48,0.58,0.38,0.48,0.58}
local ys = {-0.28,-0.28,-0.28,-0.18,-0.18,-0.18}
for i,c in ipairs(colors) do
    local block = shape(sim.primitiveshape_cuboid, {0.055,0.055,0.055}, {xs[i],ys[i],0.89}, c[2], 'block_'..c[1], 0.05)
    sim.setObjectInt32Param(block, sim.shapeintparam_static, 0)
    sim.setObjectInt32Param(block, sim.shapeintparam_respondable, 1)
end
sim.addLog(sim.verbosity_scriptinfos, 'Robotic arm scene objects created. Save scene as simulation/scene.ttt.')
