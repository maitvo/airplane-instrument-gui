var fancyText = document.getElementById('fancy');
var intervalTime = 150; // milliseconds between character deletion/insertion
var initialPause = 1000; // give the person viewing the website some time to view our site
var callbackPause = 500; // how long we'll wait before we go to the next type-text

function deleteContent(callback) {

    var intervalID = setInterval(function() {
        if (fancyText.innerHTML.length == 0) {
            clearInterval(intervalID);

            if (callback) {
                setTimeout(callback, callbackPause);
            }
        }

        fancyText.innerHTML = fancyText.innerHTML.substring(0, fancyText.innerHTML.length - 1);
    }, intervalTime);
}

function addContent(contentToAdd, callback) {
    var currIdx = 0;

    var intervalID = setInterval(function() {
        if (currIdx == contentToAdd.length) {
            clearInterval(intervalID);

            if (callback) {
                setTimeout(callback, callbackPause);
            }
        }

        fancyText.innerHTML = contentToAdd.substring(0, currIdx);
        currIdx++;
    }, intervalTime);
}

setTimeout(function() {

    deleteContent(function() {
        addContent("Airplane Instrument Visualization");
    })

    
    
}, initialPause);
