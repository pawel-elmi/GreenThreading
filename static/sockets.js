

$(document).ready(function () {
    var socket = io.connect('http://127.0.0.1:5000',
    {
        //upgrade: false,
        //transports: ['websocket']
    });


    socket.on('connect', function () {
        socket.send('user connected');
    })

    socket.on('message', function (msg) {
        $("#CH1").append("eloszka!");
        console.log('received msg',msg);
        console.log(socket.id);
    })

    socket.on('someEvent', function (msg) {
        //$("#CH2").append(msg);
        document.getElementById("CH2").innerHTML=msg;
        console.log('received msg',msg);
    })
    
    $('#sendbutton').on('click',function () {
        socket.send($('#myMessage').val());
        //$('#myMessage').val('');
    })
})
