#!/usr/bin/env node

'use strict';

var Service = require('./service');


var sonar = new Service({
  name: 'sonar',
  topic: '/sensors/range',
  op: 'subscribe',
  msgType: 'sensor_msgs/Range',
  serverTopic: 'sensors/sonar'
}, function(msg) {
  if (msg.header.frame_id === 'right_sonar_frame') {
    return {
      right: msg.range,
      left: null
    };
  } else {
    return {
      right: null,
      left: msg.range
    };
  }
});

sonar.start();
