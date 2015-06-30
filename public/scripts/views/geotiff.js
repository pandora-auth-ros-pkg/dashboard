'use strict';

var Backbone = require('backbone');
var Dispatcher = require('../dispatcher');
var ioClient = require('../ros-events');

/**
 * View template.
 */

var geotiffTemplate = require('../templates/geotiff.hbs');


var GeotiffView = Backbone.View.extend({

  el: '#geotiff',

  template: geotiffTemplate,

  events: {
    'click #request-geotiff': 'requestGeotiff',
    'click #clear-fields': 'clearFields'
  },

  initialize: function() {
    console.log('View geotiff initialized');
    console.log(this);
  },

  clearFields: function() {
    console.log('Clearing all the fields.');
  },

  requestGeotiff: function() {
    var fileName = $('input#filename-input').val();

    console.log('Sending geotiff request.');
    ioClient.emit('web/geotiff/request', fileName);
  },

  render: function() {

    this.$el.html(this.template());

    return this;
  }

});


module.exports = GeotiffView;
