#!/usr/bin/env node

'use strict';

var masterIP = require('../utils/env').getRosMasterIP();
var rosBridgePort = require('../utils/env').rosBridgePort;

/**
 * Service info.
 */

var SERVICE_NAME = 'imu';
var SUBSCRIBED_TOPIC = '/sensors/imu_rpy';

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
  type: 'pandora_sensor_msgs/ImuRPY'
};

ws.on('open', function() {
  console.log('Service ' + SERVICE_NAME + ' connected to ROS Bridge.');
  ws.send(JSON.stringify(sub));
});

/**
 * Receive data from the Topic.
 *
 * @param data.msg.imu Float - the co2 percentage.
 */

ws.on('message', function(chunk) {
  var data = JSON.parse(chunk.toString());
  var values = [
    {
      name: 'roll',
      value: data.msg.roll.toFixed(2)
    },
    {
      name: 'pitch',
      value: data.msg.pitch.toFixed(2)
    },
    {
      name: 'yaw',
      value: data.msg.yaw.toFixed(3)
    }
  ];
  console.log(values);
  socket.emit('service/sensors/imu', values);
});
