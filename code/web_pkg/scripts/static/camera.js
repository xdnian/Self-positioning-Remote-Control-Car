CAR_URI = 'http://192.168.1.31:2333'

$(document).ready(function() {
    $(".button").bind("contextmenu",function(e){   
          return false;   
    });

    $("#cam_left").mousedown(function(){
        $.post(CAR_URI+'/camera_rotate', {'action': 'left'}, function(result){
            console.log(result);
        });
    });
    $("#cam_left").mouseup(function(){
        $.post(CAR_URI+'/camera_rotate', {'action': 'stop'}, function(result){
            console.log(result);
        });
    });
    $("#cam_right").mousedown(function(){
        $.post(CAR_URI+'/camera_rotate', {'action': 'right'}, function(result){
            console.log(result);
        });
    });
    $("#cam_right").mouseup(function(){
        $.post(CAR_URI+'/camera_rotate', {'action': 'stop'}, function(result){
            console.log(result);
        });
    });
    $("#cam_reset").click(function(){
        $.post(CAR_URI+'/camera_rotate', {'action': 'reset'}, function(result){
            console.log(result);
        });
    });

    $(document).keydown(function(e) {
        switch(e.which) {
            case 37: // left
            $("#cam_left").mousedown()
            break;

            case 39: // right
            $("#cam_right").mousedown()
            break;

            default: return; // exit this handler for other keys
        }
        e.preventDefault(); // prevent the default action (scroll / move caret)
    });

    $(document).keyup(function(e) {
        switch(e.which) {
            case 37: // left
            $("#cam_left").mouseup()
            break;

            case 39: // right
            $("#cam_right").mouseup()
            break;

            default: return; // exit this handler for other keys
        }
        e.preventDefault(); // prevent the default action (scroll / move caret)
    });
});