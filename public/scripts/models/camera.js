'use strict';

var Backbone = require('backbone');
var ioClient = require('../ros-events.js');
var Dispatcher = require('../dispatcher');

/**
 * Model for the Camera indicator.
 *
 * @param name String - name of the camera.
 * @param topic String - topic to listen to for the image stream.
 * @param dispatcherTopic String - topic to notify the rest of the app.
 */

var Camera = Backbone.Model.extend({

  defaults: {
    name: 'Camera',
    streamServer: 'localhost',
    streamPort: 8080,
    videoWidth: 450,
    videoHeight: 340,
    videoQuality: 50,
    topic: '/kinect/rgb/image_raw'
  },

  /**
   * Register events on the event dispatcher.'
   */

  initialize: function() {
    var _this = this;
    this.set({'dispatcherTopic': this.get('name').toLowerCase() + ':change'});

    /**
     * Notify other parts of the app for the Camera change.
     */

    this.on('change', function() {
      Dispatcher.trigger(_this.get('dispatcherTopic'), _this.attributes);
    });
  },

  reset: function() {
    console.log('Model reseted.');
    this.set(this.defaults);
  }

});

module.exports = Camera;
