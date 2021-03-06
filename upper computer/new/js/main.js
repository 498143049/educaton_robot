var websocket = null;
var wsTimeout = null;
var wsTryOpen = false;

var floatData = new Float32Array();
var hasNewData = false;
var refreshInterval = null;
var series = null;

var maxTime = 20;
var dropData = 10;
var deviceTime = -1;

function setWsTimeout() {
    clearWsTimeout();
    if (websocket != null) {	
        if (websocket.readyState == websocket.CONNECTING) {
            wsTimeout = setTimeout("wsOnOpenTimeout()", parseInt($("#openTimeout")[0].value) * 1000);
        } else if (websocket.readyState == websocket.OPEN) {
            wsTimeout = setTimeout("wsOnNodataTimeout()", parseInt($("#nodataTimeout")[0].value) * 1000);
        } else if (websocket.readyState == websocket.CLOSING) {
            wsTimeout = setTimeout("wsOnCloseTimeout()", parseInt($("#closeTimeout")[0].value) * 1000);
        }
    }
}

function clearWsTimeout() {
    if (wsTimeout != null) {
        clearTimeout(wsTimeout);
        wsTimeout = null;
    }
}

function wsOnOpen(evt) {
    clearWsTimeout();
    console.log("已连接");
    $("#status").html("已连接");
    $("#status").attr("status", "connected");
    setWsTimeout();
}

function pageRender() {
    stats.begin();
    if (deviceData.length > 0) {
        var data = deviceData.get(-1); 
        for (var i = 0; i < deviceData.frameLength; i++) {
            $("#data" + i).html(data[i].toFixed(4));
        }
        $("#data_count").html(deviceData.length.toString());
        $("#car-time").html(data.time.toFixed(0));
        $("#car-speed").html(((data[11] + data[10]) * 50).toFixed(0));
        var direction = data.euler[2].toFixed(0);
        if (0 <= direction && direction < 45) {
            $("#car-direction").html("东偏北" + direction + "度");
        } else if (45 <= direction && direction < 90) {
            $("#car-direction").html("北偏东" + (90 - direction) + "度");
        } else if (90 <= direction && direction <135) {
            $("#car-direction").html("北偏西" + (direction - 90) + "度");
        } else if (135 <= direction) {
            $("#car-direction").html("西偏北" + (180 - direction) + "度");
        } else if (-45 <= direction && direction < 0) {
            $("#car-direction").html("东偏南" + (-direction) + "度");
        } else if (-90 <= direction && direction < -45) {
            $("#car-direction").html("南偏东" + (90 + direction) + "度");
        } else if (-135 <= direction && direction < -90) {
            $("#car-direction").html("南偏西" + (-direction - 90) + "度");
        } else if (direction < -135) {
            $("#car-direction").html("西偏南" + (180 + direction) + "度");
        } 
        plot2d.render();
        plot2d2.render();
    }
    stats.end();
    requestAnimationFrame(pageRender);
}

var isFinished = true;
function wsOnMessage(evt) {
    clearWsTimeout();
    if (typeof evt.data == "object" && evt.data instanceof Blob) {
        // console.log("收到二进制数据");
        if (evt.data.size % (deviceData.frameLength * 4) == 0) {
            if (isFinished) {
                isFinished = false;
                var fileReader = new FileReader();
                fileReader.onload = function() {
                    deviceData.push(this.result);
                    isFinished = true;
                };
                fileReader.readAsArrayBuffer(evt.data);
            } else {
                console.log("lost pkg");
            }
        } else {
            console.log("error binary frame", evt.data.size);
        }
    } else if (typeof evt.data == "string") {
        // console.log("收到文本数据");
        if (evt.data[0] == '{') {
            try {
                var response = JSON.parse(evt.data);
                for (k in response) {
                    if (k == "runonce") {
                        console.log("runonce command \"" + response[k] + "\" confirmed!!!");
                    } else {
                        remoteConfirm[k] = response[k];
                    }
                }
            } catch (e) {
                console.log(evt.data);
            }
        } else {
            $("#textFrame").html(evt.data);
        }
    }
    setWsTimeout();
}
function wsOnError(evt) {
    console.log("发生错误");
}
function wsOnClose(evt) {
    if (websocket.readyState == websocket.CONNECTING || 
        websocket.readyState == websocket.OPEN) {
        throw("unexpected ws close event");
    }
    clearWsTimeout();
    console.log("已断开");
    $("#status").html("已断开");
    $("#status").attr("status", "not-connected");
    websocket = null;
    if (wsTryOpen) {
        wsOnOpening();
    } else {
        $("#wsServer").prop('disabled', false);
        $("#openTimeout").prop('disabled', false);
        $("#closeTimeout").prop('disabled', false);
        $("#nodataTimeout").prop('disabled', false);
    }
}

function wsOnOpening() {
    clearWsTimeout();
    console.log("连接中");
    $("#status").html("连接中");
    $("#status").attr("status", "pending");
    websocket = new WebSocket("ws://" + $("#wsServer")[0].value); 
    websocket.onopen = wsOnOpen;
    websocket.onmessage = wsOnMessage;
    websocket.onerror = wsOnError;
    websocket.onclose = wsOnClose;
    setWsTimeout();
}
function wsOnClosing() {
    clearWsTimeout();
    console.log("关闭中");
    $("#status").html("关闭中");
    $("#status").attr("status", "pending");
    websocket.close();
    setWsTimeout();
}
function wsOnOpenTimeout() {
    clearWsTimeout();
    if (websocket != null && websocket.readyState == websocket.CONNECTING) {
        console.log("连接超时");
        $("#status").html("连接超时");
        $("#status").attr("status", "error");
        wsOnClosing();
    }
}
function wsOnNodataTimeout() {
    clearWsTimeout();
    if (websocket != null && websocket.readyState == websocket.OPEN) {
        console.log("无数据超时");
        $("#status").html("无数据超时");
        $("#status").attr("status", "error");
        wsOnClosing();
    }
}
function wsOnCloseTimeout() {
    clearWsTimeout();
    if (websocket != null && websocket.readyState == websocket.CLOSING) {
        console.log("关闭超时");
        $("#status").html("关闭超时");
        $("#status").attr("status", "error");
        websocket = null;
        if (wsTryOpen) {
            wsOnOpening();
        } else {
            $("#wsServer").prop('disabled', false);
            $("#openTimeout").prop('disabled', false);
            $("#closeTimeout").prop('disabled', false);
            $("#nodataTimeout").prop('disabled', false);
        }
    }
}

function dataRefreshHandler() {
    if (hasNewData) {
        hasNewData = false;
    }
}

// 上位机向下位机发送指令说明: 
// 包含了与下位机 同步控制变量 和 发送一次性控制指令 的功能；
// 指令为json格式, 下位机在收到后会以文本帧的形式响应相同的json指令；
// 特别的, 指令中的runonce字段, 将在下位机运行一次。
// 例如发送 {"runonce":"stop"} 下位机将急停, 并回复{"runonce":"stop", ...其他控制量}
var remoteControl = {speed:0,speeddiff:0,robot:0,claw:0};
var remoteConfirm = {speed:0,speeddiff:0,robot:0,claw:0};
function updateRemote() {
    if (websocket != null 
        && websocket.readyState == websocket.OPEN) 
    {
        if (websocket.bufferedAmount < 128) {
            var needUpdate = false;
            for (k in remoteControl) {
                if (remoteControl[k] != remoteConfirm[k]) {
                    needUpdate = true;
                    break;
                }
            }
            if (needUpdate) {
                var cmd = JSON.stringify(remoteControl);
                console.log(cmd);
                websocket.send(cmd);
            }
        }
    }
} 
function setRemote(k, v) {
    if (remoteControl[k] != v) {
        remoteControl[k] = v;
        if(k=="speed")
        {
            $("#" + k)[0].value = v;
        }
        updateRemote();
    }
}
function resetArmRetset(){
    var cmd = {runonce: "armreset"};
    websocket.send(JSON.stringify(cmd));
}
function resetRemote() {
    console.log("复位中");
    $("#status").html("复位中");
    $("#status").attr("status", "pending");
    clearWsTimeout();
    var cmd = {runonce: "reset"};
    websocket.send(JSON.stringify(cmd));
    setTimeout(function() {
        websocket.close();
        websocket = null;
        if (wsTryOpen) {
            wsOnOpening();
        } else {
            $("#wsServer").prop('disabled', false);
            $("#openTimeout").prop('disabled', false);
            $("#closeTimeout").prop('disabled', false);
            $("#nodataTimeout").prop('disabled', false);
        }
    }, 500);
}
// 急停
function emergencyStop() {
    remoteControl = {speed:0,speeddiff:0,robot:0,claw:0};
    var cmd = {runonce: "stop"};
    websocket.send(JSON.stringify(cmd));
}


function CalumniateNum(id,x,y,unitx,unity,rowNum){
    var height= $(id).offset().top-0.1; //防止边界问题加+0.1
    var Width= $(id).offset().left-0.1; //防止边界问题
    var numx=parseInt((x-Width)/unitx)+1;
    var numy=parseInt((y-height)/unity);
    return  numx+rowNum*numy
}
var targedSpeed=0.1;
var targedSpeeddiffer=0.1;
//定义角度处理函数
var orientationspeed = 0.015;
function orientationHandler(event) {  
    var z = 5;
    if (event.beta < -z) {
        // 前进
        var val = (-z - event.beta) * orientationspeed;
        setRemote("speed", val.toFixed(2));
    } else if (event.beta > z) {
        // 后退
        var val = (event.beta - z) * -orientationspeed;
        setRemote("speed", val.toFixed(2));
    } else if (event.beta < z && event.beta > -z) {
        setRemote("speed", 0);
    }
    if (event.gamma < -z) {
        // 逆时针旋转
        var val = (-z - event.gamma) * orientationspeed;
        setRemote("speeddiff", val.toFixed(2));
    } else if (event.gamma > z) {
        // 顺时针旋转
        var val = (event.gamma - z) * -orientationspeed;
        setRemote("speeddiff", val.toFixed(2));
    } else if (event.gamma < z && event.gamma > -z) {
        setRemote("speeddiff", 0);
    }
}
function Deal(num)
{
    switch(num)
    {
        case 1:
            setRemote("speed", targedSpeed);
            setRemote("speeddiff",targedSpeeddiffer);
         break;
         case 2:
            setRemote("speed", targedSpeed);
            setRemote("speeddiff",0);
         break;
          case 3:
            setRemote("speed", targedSpeed);
            setRemote("speeddiff",-targedSpeeddiffer);
         break;
          case 4:
            setRemote("speed", 0);
            setRemote("speeddiff",targedSpeeddiffer*1.5);
         break;
          case 5:
            setRemote("speed", 0);
            setRemote("speeddiff",0);
         break;
          case 6:
            setRemote("speed", 0);
            setRemote("speeddiff",-targedSpeeddiffer*1.5);
         break;
          case 7:
            setRemote("speed", -targedSpeed);
            setRemote("speeddiff",-targedSpeeddiffer);
         break;
          case 8:
            setRemote("speed", -targedSpeed);
            setRemote("speeddiff",0);
         break;
          case 9:
            setRemote("speed", -targedSpeed);
            setRemote("speeddiff",targedSpeeddiffer);
         break;
         default:
         break;
    }
}
function Deal_ARM(num)
{
    //处理前发生1个停止请求 
    setRemote("robot",num);
}
function keyDownHandler(event) {
    if (event.keyCode == 37) { // 左
        setRemote("speeddiff", targedSpeeddiffer);
        return false;
    } 
    if (event.keyCode == 39) { // 右
        setRemote("speeddiff", -targedSpeeddiffer);
        return false;
    } 
    if (event.keyCode == 38) { // 上
        setRemote("speed", targedSpeed);
        return false;
    } 
    if (event.keyCode == 40) { // 下
        setRemote("speed", -targedSpeed);
        return false;
    }
}
function keyUpHandler(event) {
    if (event.keyCode == 37 || event.keyCode == 39) { // 右
        setRemote("speeddiff", 0);
    }
    if (event.keyCode == 38 || event.keyCode == 40) { // 下
        setRemote("speed", 0);
    }
}


var currentnum;
var currentnum_robot;
function touchmoving(event){
    if(event.type=="mousemove")
    {
       var x=event.pageX;
       var y=event.pageY;
    }
    else
    {
        var x=event.originalEvent.touches[0].pageX;
        var y=event.originalEvent.touches[0].pageY;
    }
    var tempNum=CalumniateNum('#oprater',x,y,60,60,3);
    if(currentnum!=tempNum)
    {
        $('#oprater span:nth-child('+currentnum+')').css('background-color','white')
        $('#oprater span:nth-child('+tempNum+')').css('background-color','black')
        currentnum=tempNum;
        Deal(currentnum);
        console.log(currentnum);
    }
}
function touchmoving_RobotArm(event){
    if(event.type=="mousemove")
    {
       var x=event.pageX;
       var y=event.pageY;
    }
    else
    {
        var x=event.originalEvent.touches[0].pageX;
        var y=event.originalEvent.touches[0].pageY;
    }
    var tempNum=CalumniateNum('#ArmOperate',x,y,40,40,2);
    if(currentnum_robot!=tempNum)
    {
        $('#ArmOperate span:nth-child('+currentnum_robot+')').css('background-color','white')
        $('#ArmOperate span:nth-child('+tempNum+')').css('background-color','black')
        currentnum_robot=tempNum;
        Deal_ARM(currentnum_robot);
        console.log(currentnum_robot);
    }
}
function touchStart(event){
    
    if(event.type=="mousedown")
    {
       var x=event.pageX;
       var y=event.pageY;
    }
    else
    {
        var x=event.originalEvent.touches[0].pageX;
        var y=event.originalEvent.touches[0].pageY;
    }
    currentnum=CalumniateNum('#oprater',x,y,60,60,3);
    $('#oprater span:nth-child('+currentnum+')').css('background-color','black');
    Deal(currentnum);
    console.log(currentnum);
    $("#oprater").bind('touchmove',touchmoving);  //注册移动事件
    $("#oprater").bind('mousemove',touchmoving);  //注册移动事件
}
function touchStart_RobotArm(event){
    event.preventDefault(); //取消默认事件
    if(event.type=="mousedown")
    {
       var x=event.pageX;
       var y=event.pageY;
    }
    else
    {
        var x=event.originalEvent.touches[0].pageX;
        var y=event.originalEvent.touches[0].pageY;
    }
    currentnum_robot=CalumniateNum('#ArmOperate',x,y,40,40,2);
    $('#ArmOperate span:nth-child('+currentnum_robot+')').css('background-color','black')
    Deal_ARM(currentnum_robot);
    console.log(currentnum_robot);
    $("#ArmOperate").bind('touchmove',touchmoving_RobotArm);  //注册移动事件
    $("#ArmOperate").bind('mousemove',touchmoving_RobotArm);  //注册移动事件
}
function touchEnd()
{
     $("#oprater").unbind('touchmove',touchmoving);  //注册移动事件
      $("#oprater").unbind('mousemove',touchmoving);  //注册移动事件
     $('#oprater span').css('background-color','white');
     Deal(5);  //速度清0
}
function touchEnd_RobotArm()
{
    $("#ArmOperate").unbind('touchmove',touchmoving_RobotArm);  //注册移动事件
    $("#ArmOperate").unbind('mousemove',touchmoving_RobotArm);  //注册移动事件
    $('#ArmOperate span').css('background-color','white');
    Deal_ARM(0);  //速度清0
}
function DisplaySet(type,objectF)
{

        if(objectF=="car")
        {
             $(".postionCar").fadeToggle("slow");
             $(".direction").fadeToggle("slow");
              $(".speedtext").fadeToggle("slow");
        }
        else
        {
             $(".postionArm").fadeToggle("slow");
             $(".RobotArm").fadeToggle("slow");
        }
    
}

// 获取url参数
function getQueryString(name) {
    var reg = new RegExp("(^|&)" + name + "=([^&]*)(&|$)", "i");
    var r = window.location.search.substr(1).match(reg);
    if (r != null) return unescape(r[2]); return null;
}


var plot;
var plot2d2;
var plot2d;
var clickTime=0;
var mainType;
var slider;
$(document).ready(function(){

    // 当url中含有debug=1参数时, 显示class=debug的dom
    //默认IP为192.168.123.63 
   
    if (getQueryString("debug") == "1") {
        $(".debug").show();
    }

     if(getQueryString("type") == "robot")
     {
            $(".robot").show();
            $("#wsServer").val("192.168.123.63");
            mainType="robot";
            $(".postionCar").hide();
            $(".postionArm").hide();
          //  $(".RobotArm").hide()
           $("link[rel='shortcut icon']").attr("href","favicon2.ico");
           $("link[rel=icon]").attr("href","favicon2.ico");
           $("link[rel=apple-touch-icon-precomposed]").attr("href","favicon2.ico");
           $("link[rel=apple-touch-startup-image]").attr("href","startup2.png");
          
           $("title").html("服务机器人"); 
            targedSpeed=0.15;
            targedSpeeddiffer=0.15;
      }
      else
     {
            //默认为car
            $(".car").show();
            $(".postionCar").hide();
            $("#wsServer").val("192.168.123.15");
            mainType="car";
            targedSpeed=0.15;
            targedSpeeddiffer=0.15;
     }
    if(getQueryString('ip')!=null)
    {
        $("#wsServer").val(getQueryString('ip'));
    }
    
  slider =  $("#ex5").slider({
	reversed : true,
    formatter: function(value) {
		return  value;
	},
    tooltip_position: 'left'
    });
    slider.on("slide", function(slideEvt) {
        targedSpeed=slideEvt.value;
        targedSpeeddiffer=slideEvt.value;
    });

    $(function(argument) {
      $('#angleControl').bootstrapSwitch();
      $('#clawControl').bootstrapSwitch();
      //先发送爪子闭合(保证是张开)
    });
    $('#clawControl').on('switchChange.bootstrapSwitch', function (e, data) {
        if(data==false)
        {
            setRemote("claw",0);
        }
        else
        { 
            setRemote("claw",1);
        }
     });
    //增加事件
    $('#angleControl').on('switchChange.bootstrapSwitch', function (e, data) {
     
        if(data==false)
        {
            if (window.DeviceOrientationEvent) {  
                 window.removeEventListener("deviceorientation", orientationHandler, false); 
            }
            //同时关闭
            setRemote("speed", 0);
            setRemote("speeddiff",0);
            console.log("close");
        }
        else
        {
            if (window.DeviceOrientationEvent) {  
                 window.addEventListener("deviceorientation", orientationHandler, false); 
            }
            console.log("open");
        }

    });

     $(".postionArm").bind('click',function(){DisplaySet("head","arm");});
     $(".postionCar").bind('click',function(){DisplaySet("head","car");});
     $(".RobotArm>.panel-heading").bind('click',function(){DisplaySet("body","arm");});
     $(".direction>.panel-heading").bind('click',function(){DisplaySet("body","car");});

     

    //添加move事件
    $("#oprater").bind('touchstart',touchStart);
    $("#oprater").bind('touchend',touchEnd);
    $('#oprater').bind('mousedown',touchStart);
    $('#oprater').bind('mouseup',touchEnd);
    
    $("#ArmOperate").bind('touchstart',touchStart_RobotArm);
    $("#ArmOperate").bind('touchend',touchEnd_RobotArm);
    $("#ArmOperate").bind('mousedown',touchStart_RobotArm);
    $("#ArmOperate").bind('mouseup',touchEnd_RobotArm);

    // 初始化数据存储
    deviceData = new DeviceData();

    $.ajaxSetup({cache:false});
    $("#connect").click(function() {
        wsTryOpen = true;
        $("#wsServer").prop('disabled', true);
        $("#openTimeout").prop('disabled', true);
        $("#closeTimeout").prop('disabled', true);
        $("#nodataTimeout").prop('disabled', true);
        $("#connect").hide();
        $("#disconnect").show();
        $("#emergencyStop").prop("disabled", false);
        wsOnOpening();
        refreshInterval = setTimeout(dataRefreshHandler, 1);
    });
    $("#disconnect").click(function() {
        clearTimeout(refreshInterval);
        wsTryOpen = false;
        $("#connect").show();
        $("#disconnect").hide();
        $("#emergencyStop").prop("disabled", true);
        wsOnClosing();
    });
    $(document).keydown(keyDownHandler);
    $(document).keyup(keyUpHandler);

    
    stats = new Stats();
    stats.showPanel( 0 ); // 0: fps, 1: ms, 2: mb, 3+: custom
    //document.body.appendChild( stats.dom );
    
    // 强制设定路径图为边长不大于500px的正方形, 并居中
    $("#plot-route").height($("#plot-route").width());

    plot2d = new RealtimePlot1D($("#plot-container"), deviceData, 20);
    //plot2d.addDataIndex(22, 0xFF0000);
    //plot2d.addDataIndex(31, 0x0000FF);
    //plot2d.addDataIndex(32, 0x00FF00);
    //plot2d.addDataIndex(23, 0xFF9900);
    plot2d2 = new RealtimePlot2D($("#plot-route"), deviceData, 0.5, true, -0.25, 1.75, -0.75);
    // plot2d2.addDataIndex([28, 29], 0xFF0000);
    if(mainType=="robot")
    {
        plot2d2.setBackround("./pic/2.jpg", 2, 2, 0.75, -0.75);
    }
    else
    {
         plot2d2.setBackround("./pic/1.jpg", 2, 2, 0.75, -0.75);
    }
    plot2d2.setTarget("./pic/car-icon.png", 0.4, 0.3, 31, 32, 15);
    plot2d2.addDataIndex([31, 32], 0x00FF00);
    // plot2d2.addDataIndex([34, 35], 0xFF9900);

    var lineColors = ["FF0000", "FF9900", "00FF00", "FFFF00", "0000FF", "9900FF"];
    var colorIndex = 0;
    for (var i = 0; i < deviceData.frameLength; i++) {
        $("#data" + i).click(function() {
            var index = Number($(this).attr("id").slice(4, 100));
            if ($(this).css("background-color") == "rgba(0, 0, 0, 0)") {
                var color = lineColors[colorIndex++];
                colorIndex %= lineColors.length;
                $(this).css("background-color", "#" + color);
                plot2d.addDataIndex(index, Number("0x" + color));
                console.log("add line", index, color);
            } else {
                console.log("del line", index);
                $(this).css("background-color", "rgba(0, 0, 0, 0)");
                plot2d.delDataIndex(index);
            }
        });
        $("#data" + i).css("text-align", "right");
        $("#data_count").css("text-align", "right");
    }
    $("#data10").click();
    $("#data11").click();

    if (getQueryString("debug") != "1") {
        // 根据debug隐藏坐标轴刻度
        $(".xlabel").hide();
        $(".ylabel").hide();
    }

    
    setInterval(updateRemote, 200);
    requestAnimationFrame(pageRender);
});
