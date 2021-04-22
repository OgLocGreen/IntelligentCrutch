// JavaScript Document

// Create a client instance: Broker, Port, Websocket Path, Client ID
client = new Paho.MQTT.Client("mqtt.eclipse.org", Number(80), "/mqtt", "clientId");

// set callback handlers
client.onConnectionLost = function (responseObject) {
    console.log("Connection Lost: "+responseObject.errorMessage);
}

client.onMessageArrived = function (message) {
  console.log("Message Arrived: "+message.payloadString);
}

// Called when the connection is made
function onConnect(){
	console.log("Connected!");
}

// Connect the client, providing an onConnect callback
client.connect({
	onSuccess: onConnect
});
	

function startConnect() {

// Publish a Message
var messagepayloadjson = new Object();
messagepayloadjson.pt= document.getElementById('txtbox_Text').value;
messagepayloadjson.br= document.getElementById('txtbox_Brightness').value;
messagepayloadjson.r= document.getElementById('txtbox_ColorR').value;
messagepayloadjson.g= document.getElementById('txtbox_ColorG').value;
messagepayloadjson.b= document.getElementById('txtbox_ColorB').value;

var messagepayloadstring = JSON.stringify(messagepayloadjson);
var message = new Paho.MQTT.Message(messagepayloadstring);
message.destinationName = "8x8-WebApp/TextErzeugung";
message.qos = 0;
client.send(message);
	
}


function ToTapToLightPage(){
	window.location.replace("TapToLight.html");
}

function ToTapToTextGenerator(){
	window.location.replace("TapToTextGenerator.html");
}

function ToTapToLaunchpad(){
	window.location.replace("TapToLaunchpad.html");
}

function ToTapToSettings(){
	window.location.replace("TapToSettings.html");
}

function ToTapToHome(){
	window.location.replace("WebApp8x8.html");
}


