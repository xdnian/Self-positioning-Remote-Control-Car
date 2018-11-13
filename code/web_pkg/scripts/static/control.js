CAR_URI = 'http://192.168.1.31:2333'
DIS_MAP = ["<p>Normal</p>", "<p style='background:#e6770a'>Close</p>", "<p style='background:red'>Too close!</p>"]

$(document).ready(function() {
    pos = $('#ground').position()
    $("#car").hide()
    $("#car").append('<img src="./car.png">');

    setInterval(function(){
        $.get(CAR_URI+'/car_info', function(results){
            $("#car").show()
            console.log(results);
            xcoor = results['coordinate'][0]*50 + pos.left + 3;
            ycoor = results['coordinate'][1]*50 + pos.top + 3;
            degree = results['coordinate'][2]
            $("#car").css({
                "top": ycoor + "px",
                "left": xcoor + "px",
                '-ms-transform': 'rotate('+degree+'deg)',
                '-webkit-transform': 'rotate('+degree+'deg)',
                ' transform': 'rotate('+degree+'deg)'
            });
            $("#front").html(DIS_MAP[results['collision'][0]]);
            $("#right").html(DIS_MAP[results['collision'][1]]);
            $("#back").html(DIS_MAP[results['collision'][2]]);
            $("#left").html(DIS_MAP[results['collision'][3]]);
        });
    }, 500);
});