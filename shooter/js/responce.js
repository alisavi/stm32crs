var cvs = document.getElementById("canvas");
var ctx = cvs.getContext("2d");
var centX = cvs.width/2;
var centY = cvs.height/2;
var radius = 30;
var xPos = cvs.width/4;
var yPos = cvs.height/4;
var aimxy = {x: centX, y: centY};



var bird = new Image();
var bg = new Image();
var aim = new Image();
// var pipeUp = new Image();
// var pipeBottom = new Image();

bird.src = "/img/bird.png";
bg.src = "img/scene.png";
aim.src = "img/aim.png";
// pipeUp.src = "img/pipeUp.png";
// pipeBottom.src = "img/pipeBottom.png";
function circ (x = cvs.width/2, y = cvs.height/2, radius) {
    ctx.beginPath()
    ctx.arc(x, y, radius, 0, 2*Math.PI, false);
    ctx.fillStyle = 'yellow';
    ctx.fill();
    ctx.lineWidth = 5;
    ctx.strokeStyle = 'red';
    ctx.stroke();
}

function drawCanv() {
    ctx.fillStyle = "#3cbcfd";
    ctx.fillRect(0, 0, cvs.width, cvs.height);
}

function draw(aimxy) {
    // Какой-либо код
    


    ctx.drawImage(bird, 500, 600);

    ctx.drawImage(bg, 0, 0, cvs.width, cvs.height);
    

    //circ(centX/2, centY/2, radius);
    //circ(centX/2, centY/2, radius-10);
    //circ(centX/2, centY/2, radius-20);

    ctx.drawImage(aim, aimxy.x, aimxy.y);

    requestAnimationFrame(draw); // Вызов функции постоянно
}

// При нажатии на какую-либо кнопку
document.addEventListener("keydown", keyDown);
// Вызывается метод someMethod
function keyDown() {

 // Изменяем что-то в коде
}

function run() {
    const xhr = new XMLHttpRequest();
    xhr.open('GET', 'get_data');
    xhr.onload = function()
    {
    if (xhr.status === 200)
    {
        const response = xhr.responseText;
        console.log(response); // x [0, 1919], y [0, 1080]
        aimxy = JSON.parse(response);
    }
    else 
    {
        console.log('Ошибка загрузки данных: ' + xhr.status);
    }
    };
    xhr.send();
    drawCanv();
    draw(aimxy); // Вызов функции из-вне function draw()

};
drawCanv();
setInterval(run, 200);