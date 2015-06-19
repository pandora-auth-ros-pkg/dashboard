#!/usr/bin/env node

'use strict';

var masterIP = require('../utils/env').getRosMasterIP();
var rosBridgePort = require('../utils/env').rosBridgePort;

/**
 * Service info.
 */

var SERVICE_NAME = 'stateManager';
var SUBSCRIBED_TOPIC = '/robot/state/clients';
/**
 * Load dependencies.
 */

var io = require('socket.io-client');
var webSocket = require('ws');

/**
 * Set up the connections.
 */

var socket = io('http://localhost:3000');
socket.on('connect', function() {
  console.log('Service ' + SERVICE_NAME + ' connected to the socket.io server.');
});

var ws = new webSocket('ws://' + masterIP + ':' + rosBridgePort);

var sub = {
  op: 'subscribe',
  topic: SUBSCRIBED_TOPIC,
  type: 'state_manager_msgs/RobotModeMsg'
};

ws.on('open', function() {
  console.log('Service ' + SERVICE_NAME + ' connected to ROS Bridge.');
  ws.send(JSON.stringify(sub));
});

/**
 * Receive data from the Topic.
 *
 * @param data.msg.mode Number - The robot state.
 * @param data.msg.type Number - The type of request.
 */

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

ws.on('message', function(chunk) {
  var data = JSON.parse(chunk.toString());

  // If a transition has been made notify for the
  // current state.
  console.log(data.msg);
  socket.emit('service/robot/state', robotStates[data.msg.mode]);
});
