'use strict';

var Backbone = require('backbone');
var ioClient = require('../ros-events');

/**
 * View template.
 */

var csvTemplate = require('../templates/csv.hbs');
var generalAlertTemplate = require('../templates/general-alert.hbs');


var csvView = Backbone.View.extend({

  el: '#qr-csv',

  template: csvTemplate,
  alertTemplate: generalAlertTemplate,

  events: {
    'click #request-csv': 'requestCSV',
    'click #clear-fields': 'clearFields'
  },

  initialize: function() {
    console.log('View csv initialized');
    ioClient.on('web/csv/response', this.showResult.bind(this));
  },

  clearFields: function() {
    console.log('Clearing all the fields.');
    $('input#filename-input').val('');
  },

  requestCSV: function() {
    var fileName = $('input#filename-input').val();

    if (fileName === '') {
      console.log('Filename should not be empty.');
      this.showConfirmationAlert({
        type: 'warning',
        heading: 'The mission name should not be empty.'
      });
    } else {
      console.log('Sending csv request.');
      ioClient.emit('web/csv/request', fileName);
    }
  },

  showResult: function(res) {
    console.log('Received response from the csv service.');

    if (res.result === true) {
      this.showConfirmationAlert({
        type: 'success',
        heading: 'Service response',
        body: 'the csv file was saved successfully'
      });
    } else {
      var body;
      if (typeof res.values === 'string') {
        body = res.values;
      } else {
        body = 'make sure the csv service is running';
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


module.exports = csvView;
