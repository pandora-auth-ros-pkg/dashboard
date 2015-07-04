'use strict';

var Backbone = require('backbone');
var ioClient = require('../ros-events');

/**
 * View template.
 */

var geotiffTemplate = require('../templates/geotiff.hbs');
var generalAlertTemplate = require('../templates/general-alert.hbs');


var GeotiffView = Backbone.View.extend({

  el: '#geotiff',

  template: geotiffTemplate,
  alertTemplate: generalAlertTemplate,

  events: {
    'click #request-geotiff': 'requestGeotiff',
    'click #clear-fields': 'clearFields'
  },

  initialize: function() {
    console.log('View geotiff initialized');
    ioClient.on('web/geotiff/response', this.showResult.bind(this));
  },

  clearFields: function() {
    console.log('Clearing all the fields.');
    $('input#filename-input').val('');
  },

  requestGeotiff: function() {
    var fileName = $('input#filename-input').val();

    if (fileName === '') {
      console.log('Filename should not be empty.');
      this.showConfirmationAlert({
        type: 'warning',
        heading: 'The mission name should not be empty.'
      });
    } else {
      console.log('Sending geotiff request.');
      ioClient.emit('web/geotiff/request', fileName);
    }
  },

  showResult: function(res) {
    console.log('Received response from the geotiff service.');

    if (res.result === true) {
      this.showConfirmationAlert({
        type: 'success',
        heading: 'Service response',
        body: 'the geotiff map was saved successfully'
      });
    } else {
      var body;
      if (typeof res.values === 'string') {
        body = res.values;
      } else {
        body = 'make sure the geotiff service is running';
      }

      this.showConfirmationAlert({
        type: 'danger',
        heading: 'Service Error ',
        body: body
      });
    }
  },

  showConfirmationAlert: function(options) {
    var alertData = {
      alertType: options.type,
      alertHeading: options.heading,
      alertBody: options.body
    };

    this.$('#alert-placeholder').html(this.alertTemplate(alertData));
  },

  render: function() {

    this.$el.html(this.template());

    return this;
  }

});


module.exports = GeotiffView;
