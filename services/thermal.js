#!/usr/bin/env node

'use strict';

var masterIP = require('../utils/env').getRosMasterIP();
var rosBridgePort = require('../utils/env').rosBridgePort;

/**
 * Service info.
 */

var SERVICE_NAME = 'thermal';
var SUBSCRIBED_TOPIC = '/vision/thermal_direction_alert';

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
  type: 'pandora_vision_msgs/ThermalAlertVector'
};

ws.on('open', function() {
  console.log('Service ' + SERVICE_NAME + ' connected to ROS Bridge.');
  ws.send(JSON.stringify(sub));
});

/**
 * Receive data from the Topic.
 *
 * @param data.msg.name Array - Names of the temperatures for every core.
 * @param data.msg.temperature Array - Temperature values.
 *
 */

ws.on('message', function(chunk) {
  var data = JSON.parse(chunk.toString());
  console.log(data.msg.alerts[0].temperature);
  socket.emit('service/sensors/thermal', data.msg.alerts[0].temperature);
});
