#!/usr/bin/env node

'use strict';

var Service = require('./service');

var robotStates = [
  'Off',
  'Autonomous',
  'Exploration',
  'Identification',
  'Sensor Hold',
  'Semi Auto',
  'Teleoperation',
  'Sensor Test',
  'Mapping',
  'Terminating'
];

var stateManager = new Service({
  name: 'State Manager',
  topic: '/robot/state/clients',
  op: 'subscribe',
  msgType: 'state_manager_msgs/RobotModeMsg',
  serverTopic: 'robot/state'
}, function(msg) {
  return robotStates[msg.mode];
});

stateManager.start();
