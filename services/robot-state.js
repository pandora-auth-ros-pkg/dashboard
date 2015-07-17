#!/usr/bin/env node

'use strict';

var Service = require('./service');

var ROS_MASTER_IP = require('../utils/env').ROS_MASTER_IP;
var ROS_BRIDGE_PORT = require('../utils/env').ROS_BRIDGE_PORT;

var io = require('socket.io-client');
var socket = io('http://localhost:3000');
var webSocket = require('ws');
var ws =  new webSocket('ws://' + ROS_MASTER_IP + ':' + ROS_BRIDGE_PORT);

ws.on('open', function(msg) {
  console.log('Robot state changer connected to ROS bridge.');
});


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

socket.on('connect', function() {
  console.log('Robot mode change connected to the socket.io server.');
});

socket.on('service/robot/changeMode', function(mode) {
  var modeToChange = parseInt(mode);
  var msg = {
    op: 'call_service',
    service: '/robot/change_mode',
    args: [
      modeToChange
    ]
  };
  console.log(msg);

  ws.send(JSON.stringify(msg));
});

stateManager.start();
