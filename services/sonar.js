#!/usr/bin/env node

'use strict';

var masterIP = require('../utils/env').getRosMasterIP();
var rosBridgePort = require('../utils/env').rosBridgePort;

/**
 * Service info.
 */

var SERVICE_NAME = 'sonar';
var SUBSCRIBED_TOPIC = '/sensors/range';

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
  type: 'sensor_msgs/Range'
};

ws.on('open', function() {
  console.log('Service ' + SERVICE_NAME + ' connected to ROS Bridge.');
  ws.send(JSON.stringify(sub));
});

/**
 * Receive data from the Topic.
 *
 * @param data.msg.right_range Float - range from the right sonar
 * @param data.msg.left_range Float - range from the left sonar
 */

ws.on('message', function(chunk) {
  var data = JSON.parse(chunk.toString());
  if (data.msg.header.frame_id === 'right_sonar_frame') {

    // Send null for the left sonar.
    socket.emit('service/sensors/sonar', null, data.msg.range);
  } else {

    // Send null for the right sonar.
    socket.emit('service/sensors/sonar', data.msg.range, null);
  }
});
