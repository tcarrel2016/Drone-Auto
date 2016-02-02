//ardroneJimp.js

var prompt = require('prompt');
var ardrone = require('ar-drone')
//var lwip = require('./lwip-master/index.js')
var jimp = require('./jimp-master/index.js')

var client = ardrone.createClient()
var pngImage
var output
var markerX = -1
var markerY = -1
var lowBound = [0,110,165]           //BGR      r/b > 2.2 && r/g > 1 && r/g < 2.3
var highBound = [65,165,255]
var erosionFactor = 3
var count = 0
var skipSize = 2
var room = 0
var computer_room
var cafeteria = 'cafeteria';
   

/*lwip.open("./imageWithRedMarker.png", function(err, image) {
          if (err) throw err
          
          console.log("shrinking image...")
          image.scale(0.5, function(err, shrunken) {
                      if (err) throw err
                      
                      shrunken.writeFile("./ardroneJimpOutput/lwipShrunken.png", "png", ()=>{})
                      })
          console.log("done")
          })*/

//prompt.properties = {
	//room: {
		//format: 'room'
		//message: 'Please choose from the only option available... the cafeteria'
	//},
//};		

prompt.start()
 
prompt.get (['room'], function (err, result) {
	if (err) {return onErr(err);}
	console.log('Command-line input received:');
	console.log('room:' +result.room);
	if (result.room === 'cafeteria'){
		room=1;
		console.log('room variable:' + room);
		}
	else {
	}	
	
	if (room===1) {
	client.config("video:video_channel", 0)

	var pngStream = client.getPngStream()

	pngStream
	.on("error", console.log)
	.on("data", function(incoming) {
		if (count < 200) {
			 processImage(incoming)
		
			 client.stop()
		
			 if (markerX > -1 && markerY > -1) {
				if (markerX > 320 + 100) {
					client.right(0.1)
					console.log("GO RIGHT")
				}
				else if (markerX < 320 - 100) {
					client.left(0.1)
					console.log("GO LEFT")
				}
				else {
					console.log("HOVER")
				}
			 }
			 else {
			
			 }
		 
			 count++
			 console.log(String(count))
		}
		else {
			if (count > 200 || count == 200) {
				client.stop()
				client.land()
			}
		}
		})

	client.takeoff()
}
		 
});



	
//....................................................................................................

function processImage(input) {
    pngImage = input
    jimp.read(pngImage, function(err, image) {
              if (err) throw err
              //console.log("thresholding...")
              image = thresholdImage(image)
              //console.log("eroding...")
              image = erodeImage(image)
              
              //console.log("finding marker...")
              var marker = findMarker(image)
              
              if (marker[0] > -1 && marker[1] > -1) {
                image.setPixelColor(jimp.rgbaToInt(255,0,0,255),marker[0],marker[1])
                console.log("MARKER: " + String(marker[0]) + "," + String(marker[1]))
              }
              else {
                console.log("...")
              }
              
              image.write("./ardroneJimpOutput/jimpDetect.png")
              output = marker
              markerX = output[0]
              markerY = output[1]
              })
} //jkjdjdj

function thresholdImage(image) {
    for (var y = 0; y < image.bitmap.height - skipSize; y += skipSize) {
        for (var x = 0; x < image.bitmap.width - skipSize; x += skipSize) {
            var color = jimp.intToRGBA(image.getPixelColor(x,y))
            //if (color.r / color.b > 2.2 && color.r / color.g > 1 && color.r / color.g < 2.3) {
            if (color.r / color.b > 1.5 && color.r / color.g > 1.5) {
                image.setPixelColor(jimp.rgbaToInt(255,255,255,255),x,y)
            }
            else {
                image.setPixelColor(jimp.rgbaToInt(0,0,0,255),x,y)
            }
        }
    }
    
    return image
}

function erodeImage(image) {
    for (var y = 0; y < image.bitmap.height - skipSize; y += skipSize) {
        for (var x = 0; x < image.bitmap.width - skipSize; x += skipSize) {
            var color = jimp.intToRGBA(image.getPixelColor(x,y))
            var neighborColor = color
            
            if ((x-(erosionFactor*skipSize)) > 0) {
                neighborColor = jimp.intToRGBA(image.getPixelColor(x-(erosionFactor * skipSize),y))
                
                if (neighborColor.r == 255 && (x+(erosionFactor*skipSize)) < image.bitmap.width) {
                    neighborColor = jimp.intToRGBA(image.getPixelColor(x+(erosionFactor * skipSize),y))
                    
                    if (neighborColor.r == 255 && (y-(erosionFactor*skipSize)) > 0) {
                        neighborColor = jimp.intToRGBA(image.getPixelColor(x,y-(erosionFactor * skipSize)))
                        
                        if (neighborColor.r == 255 && (y+(erosionFactor*skipSize)) < image.bitmap.height) {
                            neighborColor = jimp.intToRGBA(image.getPixelColor(x,y+(erosionFactor * skipSize)))
                            
                            if (neighborColor.r == 255) {
                                image.setPixelColor(jimp.rgbaToInt(255,255,255,255),x,y)
                            }
                            else {
                                image.setPixelColor(jimp.rgbaToInt(color.r,color.g,0,255),x,y)
                            }
                        }
                        else {
                            image.setPixelColor(jimp.rgbaToInt(color.r,color.g,0,255),x,y)
                        }
                    }
                    else {
                        image.setPixelColor(jimp.rgbaToInt(color.r,color.g,0,255),x,y)
                    }
                }
                else {
                    image.setPixelColor(jimp.rgbaToInt(color.r,color.g,0,255),x,y)
                }
            }
        }
    }
    
    return image
}

/*function getBlobs(image) {
    
}

function checkLinks() {
    
}*/

function findMarker(image) {
    var sumX = 0
    var avgX = 0
    var sumY = 0
    var avgY = 0
    var count = 0
    var center
    var centerVerified = false
    
    for (var y = 0; y < image.bitmap.height - skipSize; y += skipSize) {
        for (var x = 0; x < image.bitmap.width - skipSize; x += skipSize) {
            var color = jimp.intToRGBA(image.getPixelColor(x,y))
            
            if (color.b > 0) {
                sumX += x
                sumY += y
                count++
            }
        }
    }
    
    avgX = sumX/count
    avgY = sumY/count
    
    var color = jimp.intToRGBA(image.getPixelColor(avgX, avgY))
    if (color.b > 0 && avgX - (erosionFactor * skipSize) > 0 && avgX + (erosionFactor * skipSize) < image.bitmap.width && avgY - (erosionFactor * skipSize) > 0 && avgY + (erosionFactor * skipSize) < image.bitmap.height) {
        color = jimp.intToRGBA(image.getPixelColor(avgX - (erosionFactor * skipSize), avgY))
        if (color.b > 0) {
            color = jimp.intToRGBA(image.getPixelColor(avgX + (erosionFactor * skipSize), avgY))
            if (color.b > 0) {
                color = jimp.intToRGBA(image.getPixelColor(avgX, avgY - (erosionFactor * skipSize)))
                if (color.b > 0) {
                    color = jimp.intToRGBA(image.getPixelColor(avgX, avgY + (erosionFactor * skipSize)))
                    if (color.b > 0) {
                        centerVerified = true
                    }
                }
            }
        }
    }
    
    //if (centerVerified) {
        center = [avgX, avgY]
    //}
    //else {
        //center = [-1, -1]
    //}
    
    return center
}