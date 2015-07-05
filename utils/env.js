'use strict';

var os = require('os');
var ifaces = os.networkInterfaces();

/*
 * Configuration shared among the services.
 */

var getLocalIP = function() {
  if (ifaces.wlan0 !== undefined) {
    return ifaces.wlan0[0].address;
  }
  else if (ifaces.eth0 !== undefined) {
    return ifaces.eth0[0].address;
  }
  else {
    return '127.0.0.1';
  }
};

var env = {
  SERVER_PORT: '3000',

  ROS_MASTER_IP: 'localhost',
  ROS_BRIDGE_PORT: '9090',

  VICTIM_ALERT_PORT: '6666',
  VICTIM_VALIDATION_PORT: '6667',

  LOCAL_IP: getLocalIP()
};


module.exports = env;
