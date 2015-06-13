'use strict';

var Backbone = require('backbone');
var ioClient = require('../ros-events.js');
var Dispatcher = require('../dispatcher');

/**
 * Model for the Temperature indicator.
 *
 * @param name Array - name of the cores.
 * @param temperature Array - temperatures for every core.
 * @param topic String - socket.io topic to listen to for incoming data.
 * @param dispatcherTopic String - topic to notify the rest of the app.
 */

var Temperature = Backbone.Model.extend({

  defaults: {
    dispatcherTopic: 'cpu:temperature:change',
    name: [
      'core 1',
      'core 2',
      'core 3',
      'core 4'
    ],
    temperature: [0, 0, 0, 0]
  },

  /**
   * Register events on the event dispatcher.'
   */

  initialize: function() {
    var _this = this;

    /**
     * Notify other parts of the app for the temperatures.
     */

    this.on('change:temperature', function() {
      Dispatcher.trigger(_this.get('dispatcherTopic'), _this.attributes);
    });
  },

  /**
   * Listen for incoming data from the socket.io server.
   */

  subscribe: function(topic) {
    this.set({'topic': topic || this.get('topic')});
    console.log('Listening on topic ' + this.get('topic'));

    var _this = this;
    ioClient.on(this.get('topic'), function(msg) {
      _this.set({'temperature': msg});
    });
  },

  unsubscribe: function() {
    ioClient.removeListener(this.get('topic'));
  }

});

module.exports = Temperature;
