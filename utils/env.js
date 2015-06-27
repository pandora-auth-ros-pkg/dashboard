'use strict';

/*
 * Configuration shared among the services.
 */

var env = {
  SERVER_PORT: '3000',

  ROS_MASTER_IP: 'localhost',
  ROS_BRIDGE_PORT: '9090',

  VICTIM_ALERT_PORT: '6666',
  VICTIM_VALIDATION_PORT: '6667'
};


module.exports = env;
