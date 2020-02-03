

$(document).ready(function () {
    var socket = io.connect('http://10.113.10.238:10001',
    {
        //upgrade: false,
        //transports: ['websocket']
    });
    
    socket.on('json', function (json) {
        console.log(json)
    })


    socket.on('connect', function () {
        socket.send('user connected');
    })

    socket.on('message', function (msg) {
        //$("#CH1").append("eloszka!");
        console.log('received msg',msg);
        console.log(socket.id);
    })

    socket.on('someEvent', function (msg) {
        var obj; 
        obj = JSON.parse(JSON.stringify(msg));
        //temp = msg;
        document.getElementById("CH1").innerHTML=obj.CH1;
        document.getElementById("CH2").innerHTML=obj.CH2;
        document.getElementById("CH3").innerHTML=obj.CH3;
        document.getElementById("CH4").innerHTML=obj.CH4;
        //console.log('received msg',obj);
        //console.log('received obj',obj.data);

    })
    
    $('#sendbutton').on('click',function () {
        socket.send($('#myMessage').val());
        //$('#myMessage').val('');
    })
    $('#serwis').on('click',function () {
        socket.send($('#myMessage').val());
        //$('#myMessage').val('');
    })
    $('#login').on('click',function () {
        socket.send($('#myMessage').val());
        //$('#myMessage').val('');
    })
    $('#proces').on('click',function () {
        socket.send($('#myMessage').val());
        //$('#myMessage').val('');
    })
})
