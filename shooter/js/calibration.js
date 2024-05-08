var cvs = document.getElementById("canvas");
    cvs.height = window.innerHeight;//-window.innerHeight%240;
    cvs.width = window.innerWidth;//-window.innerWidth%256;
    var ctx = cvs.getContext("2d");
    ctx.imageSmoothingEnabled = false;
    var n = new Image();
    n.src = "/img/numbers.png";
    ctx.drawImage(n, 0, 0, 40, 40, 0, 0, cvs.width, cvs.height);