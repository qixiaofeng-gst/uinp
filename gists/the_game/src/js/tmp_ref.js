window.onload = function() {

    var canvas = document.getElementById('c');
    canvas.width = 1000;//window.innerWidth;
    canvas.height = 340;//window.innerHeight;
    var ctx = canvas.getContext('2d');

    //----------------------------------------

    var Loop = function() {

        //var time = new Date().getTime();

        jsv.update(16);
        jsv.draw(ctx);

        //console.log(new Date().getTime() - time);

        requestAnimFrame(Loop);
    };

    Loop();
};