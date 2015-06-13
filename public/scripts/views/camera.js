'use strict';

var Backbone = require('backbone');
var Dispatcher = require('../dispatcher');

/**
 * View template.
 */

var cameraTemplate = require('../templates/camera.hbs');


var CameraView = Backbone.View.extend({

  el: '.camera-panel',

  topicInput: '#topic-input',
  qualityInput: '#video-quality',
  widthInput: '#video-height',
  heightInput: '#video-width',

  events: {
    'click #update-parameters': 'updateParameters',
    'click #reset-parameters': 'resetParameters'
  },

  template: cameraTemplate,

  initialize: function() {
    console.log('View camera initialized');
    Dispatcher.on(this.model.get('dispatcherTopic'), this.render, this);
  },

  updateParameters: function() {
    var topic = $('input#topic-input').val();
    var quality= $('input#video-quality').val();
    var height= $('input#video-height').val();
    var width= $('input#video-width').val();

    if (topic) {
      this.model.set({'topic': topic});
    }
    if (quality) {
      this.model.set({'videoQuality': quality});
    }
    if (height) {
      this.model.set({'videoHeight': height});
    }
    if (width) {
      this.model.set({'videoWidth': width});
    }
  },

  /**
   * Reset all the parameters of the model to the default values.
   */

  resetParameters: function() {
    console.log('Reseting the model.');
    this.model.reset();
  },

  render: function() {

    this.$el.html(this.template(this.model.toJSON()));

    return this;
  }

});


module.exports = CameraView;
