'use strict';

$('.alert').addClass('in').fadeOut(4500);

/* swap open/close side menu icons */
$('[data-toggle=collapse]').click(function(){
  // toggle icon
  $(this).find('i').toggleClass('fa-chevron-right fa-chevron-down');
});

$('#main-dashboard-content').on('click', function() {
  if (slider.isOpen()) {
    $('#slide-menu-button').trigger('click');
  }
});

$('#slide-menu .menu-item').on('click', function() {
  if (slider.isOpen()) {
    $('#slide-menu-button').trigger('click');
  }
});

$('.alert-feed button.clean-alerts').on('click', function() {
  console.log('Clearing the alerts');
  $('.alert-feed ul').children().each(function(number, item) {
    if ($(this).hasClass('victim-alert')) {
      $(this).remove();
    }
  });

  $('.alert-feed .badge').html(0);
});

$('#slide-menu-button').on('click', function() {
  var mainContent= $('#main-content');
  slider.toggle();
});
