#!/usr/bin/env node

'use strict';

var Service = require('./service');


var QRAlert = new Service({
  name: 'qrAlert',
  topic: '/data_fusion/alert_handler/qr_notification',
  op: 'subscribe',
  msgType: 'pandora_data_fusion_msgs/QrInfo',
  serverTopic: 'alert/qr'
}, function(msg) {
  return {
    id: msg.id,
    content: msg.content,
    probability: msg.probability,
    timeFound: msg.timeFound.secs,
    position: {
      frameID: msg.qrPose.header.frame_id,
      x: msg.qrPose.pose.position.x,
      y: msg.qrPose.pose.position.y,
      z: msg.qrPose.pose.position.z
    },
    qrFrameId: msg.qrFrameId
  };
});

QRAlert.start();
