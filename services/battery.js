#!/usr/bin/env node

'use strict';

var masterIP = require('../utils/env').getRosMasterIP();
var rosBridgePort = require('../utils/env').rosBridgePort;

/**
 * Service info.
 */

var SERVICE_NAME = 'batteries';
var SUBSCRIBED_TOPIC = '/sensors/battery';
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
  type: 'pandora_sensor_msgs/BatteryMsg'
};

ws.on('open', function() {
  console.log('Service ' + SERVICE_NAME + ' connected to ROS Bridge.');
  ws.send(JSON.stringify(sub));
});

/**
 * Receive data from the Topic.
 *
 * @param data.msg.name Array - Names of the batteries.
 * @param data.msg.voltage Array - Values of the batteries.
 *
 */

ws.on('message', function(chunk) {
  var data = JSON.parse(chunk.toString());
  console.log(data.msg);
  socket.emit('service/sensors/battery', data.msg);
});
