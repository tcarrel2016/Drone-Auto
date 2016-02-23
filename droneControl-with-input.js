//ardroneAutonomousControl.js
//image = 640x360
//Blob detection
//No Density
//Horizontal tracking
//Vertical tracking

var prompt = require('prompt');
var ardrone = require('ar-drone')
var jimp = require('./jimp-master/index.js')

var client = ardrone.createClient()
var pngImage
var output
var markerX = -1
var markerY = -1
var markerR = -1
var erosionFactor = 2
var count = 0
var skipSize = 3
var previousX = 0
var previousY = 0
var previousZ = 0
var blobsFound = new BlobLibrary()
var room = 0
var cafeteria = 'cafeteria'

client.config("video:video_channel", 0)

var pngStream = client.getPngStream()

prompt.start();

prompt.get (['room'], function (err, result) 
		{ 
		console.log('Command-line input received:');
		console.log('room:' +result.room);
		})
			if (result.room === 'cafeteria')
				{room=1;
				console.log('room variable:' +room)
				}
			else {
			}
			
			if (room===1)					
				{pngStream
				.on("error", console.log)
				.on("data", function(incoming) {
	
					console.log(String(count))
	
					if (count < 50 && count > 25) {
						processImage(incoming)

						client.stop()
	
						if (markerX > -1) {
							if (markerX > 320 + 80) {
								client.right(0.08)
								previousX = 1
								console.log("RIGHT")
							}
							else if (markerX < 320 - 80) {
								client.left(0.08)
								previousX = -1
								console.log("LEFT")
							}
							else {
								if (previousX < 0) {
									client.right(0.1)
								}
								else if (previousX > 0) {
									client.left(0.1)
								}
								console.log("HOVER")
								previousX = 0
							}
						}
						else {
							if (previousX < 0) {
								client.right(0.05)
							}
							else if (previousX > 0) {
								client.left(0.05)
							}
							console.log("NO X")
							previousX = 0
						}

						if (markerY > -1) {
							if (markerY > 180 + 20) {
								client.down(0.15)
								previousY = 1
								console.log("DOWN")
							}
							else if (markerY < 180 - 20) {
								client.up(0.15)
								previousY = -1
								console.log("UP")
							}
							else {
								if (previousY < 0) {
									client.down(0.2)
								}
								else if (previousY > 0) {
									client.up(0.2)
								}
								console.log("HOVER")
								previousY = 0
							}
						}
						else {
							if (previousY < 0) {
								client.down(0.1)
							}
							else if (previousY > 0) {
								client.up(0.1)
							}
							console.log("NO Y")
							previousY = 0
						}
	
				//        if (markerR > -1) {
				//            if (markerR > 45 + 5) {
				//                client.back(0.08)
				//                previousZ = -1
				//                console.log("BACK")
				//            }
				//            else if (markerR < 45 - 5) {
				//                client.front(0.08)
				//                previousZ = 1
				//                console.log("FORTH")
				//            }
				//            else {
				//                if (previousZ < 0) {
				//                    client.front(0.05)
				//                }
				//                else if (previousZ > 0) {
				//                    client.back(0.05)
				//                }
				//                console.log("HOVER")
				//                previousZ = 0
				//            }
				//        }
				//        else {
				//            if (previousZ < 0) {
				//                client.front(0.05)
				//            }
				//            else if (previousZ > 0) {
				//                client.back(0.05)
				//            }
				//            console.log("HOVER")
				//            previousZ = 0
				//        }
	
						console.log("#Blobs: " + String(blobsFound.blobs.length))
					}
					else {
						if (count > 50 || count == 50) {
							client.stop()
							client.land()
						}
					}
	
					count++
					})

				client.takeoff()
				}

//....................................................................................................

function processImage(input) {
    pngImage = input
    jimp.read(pngImage, function(err, image) {
              if (err) throw err
              image = thresholdImage(image)
              image = erodeImage(image)
              
              //var marker = findMarker(image)      THIS IS THE OLD AVERAGING FUNCTION
              
              findBlobs(image)                    //THESE ARE THE NEW BLOB FUNCTIONS
              var marker = analyzeBlobsFound()
              
              if (marker[0] > -1 && marker[1] > -1) {
                image.setPixelColor(jimp.rgbaToInt(255,0,0,255),marker[0],marker[1])
                for (var i=0; i<marker[2]; i++) {
                  if (marker[0] + i + 1 < image.bitmap.width) {
                    image.setPixelColor(jimp.rgbaToInt(255,0,0,255),marker[0]+i+1,marker[1])
                  }
                }
              }
              else {
                console.log(".........")
              }
              
              if (count % 5 == 0) {
                image.write("./ardroneAutonomousControlOutput/image_" + count + ".png")
              }
              output = marker
              markerX = output[0]
              markerY = output[1]
              markerR = output[2]
              })
}

function thresholdImage(image) {
    for (var y = 0; y < image.bitmap.height - skipSize; y += skipSize) {
        for (var x = 0; x < image.bitmap.width - skipSize; x += skipSize) {
            var color = jimp.intToRGBA(image.getPixelColor(x,y))
            //if (color.r / color.b > 2.2 && color.r / color.g > 1 && color.r / color.g < 2.3) {                                                     //YELLOW-ORANGE
            //if (color.r / color.b > 1.5 && color.r / color.g > 1.5) {                                                                              //RED
            if (color.r / color.b > (240/110)-0.2 && color.r / color.b < (240/110)+0.8 && color.r / color.g > (240/172)-0.2 && color.r / color.g < (240/172)+0.8) {     //ORANGE
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

function findBlobs(image) {
    blobsFound.blobs = []
    
    for (var y = 0; y < image.bitmap.height - skipSize; y += skipSize) {
        for (var x = 0; x < image.bitmap.width - skipSize; x += skipSize) {
            var color = jimp.intToRGBA(image.getPixelColor(x,y))
            
            if (color.b > 0 && blobsFound.blobs.length < 6) {
                blobsFound.addBlob()
                checkLinks(image, x, y, 0)
            }
        }
    }
}

function checkLinks(image, x, y, direction) {
    var inBlob = false
    
    for (var i=0; i<blobsFound.blobs.length; i++) {
        for (var j=0; j<blobsFound.blobs[i].links.length; j++) {
            if (blobsFound.blobs[i].links[j].x == x && blobsFound.blobs[i].links[j].y == y) {
                inBlob = true
            }
            if (inBlob) {
                break
            }
        }
    }
    
    if(inBlob) {
        if (blobsFound.blobs[blobsFound.blobs.length-1].links.length == 0) {
            blobsFound.blobs.pop()
        }
    }
    else {
        blobsFound.blobs[blobsFound.blobs.length-1].addLink(x,y)
        
        if (direction != 0 && y-skipSize > 0) {
            var color = jimp.intToRGBA(image.getPixelColor(x,y-skipSize))
            
            if (color.b > 0) {
                checkLinks(image, x, y-skipSize, 2)
            }
        }
        if (direction != 1 && x+skipSize < image.bitmap.width) {
            var color = jimp.intToRGBA(image.getPixelColor(x+skipSize,y))
            
            if (color.b > 0) {
                checkLinks(image, x+skipSize, y, 3)
            }
        }
        if (direction != 2 && y+skipSize < image.bitmap.height) {
            var color = jimp.intToRGBA(image.getPixelColor(x,y+skipSize))
            
            if (color.b > 0) {
                checkLinks(image, x, y+skipSize, 0)
            }
        }
        if (direction != 3 && x-skipSize > 0) {
            var color = jimp.intToRGBA(image.getPixelColor(x-skipSize,y))
            
            if (color.b > 0) {
                checkLinks(image, x-skipSize, y, 1)
            }
        }
    }
}

function analyzeBlobsFound() {
    var bestCircularity = [2]       //circularity, blob#
    bestCircularity[0] = 20
    bestCircularity[1] = 0
    
    var bestDensity = [2]           //density,     blob#
    bestDensity[0] = 0
    bestDensity[1] = 0
    
    var bestBlob = 0
    
    for (var i=0; i<blobsFound.blobs.length; i++) {
        if (blobsFound.blobs[i].links.length < 10) {
            blobsFound.blobs.splice(i,1)
            i--
        }
        else if (blobsFound.blobs[i].links.length > 0) {
            blobsFound.blobs[i].calculateCenter()
            blobsFound.blobs[i].calculateRadiusAndCircularity()
            
            var circularity = blobsFound.blobs[i].aspects[3]
            if (circularity < bestCircularity[0]) {
                bestCircularity[0] = circularity
                //bestCircularity[1] = i
                bestBlob = i
            }
            
            /*var density = blobsFound.blobs[i].aspects[4] / blobsFound.blobs[i].aspects[2]
            if (density > bestDensity[0]) {
                bestDensity[0] = density
                bestDensity[1] = i
            }*/
        }
    }
    
    if (blobsFound.blobs.length > 0) {
        /*if (bestCircularity[1] != bestDensity[1]) {
            bestBlob = i
        }
        else {
            var score1 = (blobsFound.blobs[bestCircularity[1]].aspects[4] / blobsFound.blobs[bestCircularity[1]].aspects[2]) - bestCircularity[0]
            var score2 = bestDensity[0] - blobsFound.blobs[bestDensity[1]].aspects[3]
            
            if (score1 > score2) {
                bestBlob = bestCircularity[1]
            }
            else {
                bestBlob = bestDensity[1]
            }
        }*/
        
        var markerData = [blobsFound.blobs[bestBlob].aspects[0],blobsFound.blobs[bestBlob].aspects[1],blobsFound.blobs[bestBlob].aspects[2]]
    }
    else {
        var markerData =[320,180]
        console.log("...")
    }
    return markerData
}

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
    if (avgX > 0 && avgY > 0) {
        center = [avgX, avgY]
    }
    else {
        center = [-1, -1]
    }
    //}
    //else {
        //center = [-1, -1]
    //}
    
    return center
}

function BlobLibrary() {
    this.blobs = []
}

BlobLibrary.prototype.addBlob = function() {
    this.blobs = this.blobs.concat(new Blob())
}



function Blob() {
    this.links = []
    this.aspects = [5]
    this.aspects[0] = 320
    this.aspects[1] = 200
    this.aspects[2] = 50
    this.aspects[3] = 3
    this.aspects[4] = 5
}

Blob.prototype.addLink = function(x, y) {
    this.links = this.links.concat(new Link(x, y))
}

Blob.prototype.calculateCenter = function() {
    var X = 0
    var Y = 0
    
    for (var i=0; i<this.links.length; i++) {
        X += this.links[i].x
        Y += this.links[i].y
    }
    
    X /= this.links.length
    Y /= this.links.length
    
    this.aspects[0] = X
    this.aspects[1] = Y
}

Blob.prototype.calculateRadiusAndCircularity = function() {
    var L = 0;
    var R = 0;
    var U = 0;
    var D = 0;
    
    for (var i=0; i<this.links.length; i++) {
        if (this.aspects[0] - this.links[i].x > L) {
            L = this.aspects[0] - this.links[i].x
        }
        if (this.links[i].x - this.aspects[0] > R) {
            R = this.links[i].x - this.aspects[0]
        }
        if (this.aspects[1] - this.links[i].y > U) {
            U = this.aspects[1] - this.links[i].y
        }
        if (this.links[i].y - this.aspects[1] > D) {
            D = this.links[i].y - this.aspects[1]
        }
    }
    
    this.aspects[2] = (L+R+U+D) / 4;
    
    if (L-this.aspects[2] < 0) {
        L = -1 * (L-this.aspects[2])
    }
    else {
       L = (L-this.aspects[2])
    }
    if (R-this.aspects[2] < 0) {
        R = -1 * (R-this.aspects[2])
    }
    else {
        R = (R-this.aspects[2])
    }
    if (U-this.aspects[2] < 0) {
        U = -1 * (U-this.aspects[2])
    }
    else {
        U = (U-this.aspects[2])
    }
    if (D-this.aspects[2] < 0) {
        D = -1 * (D-this.aspects[2])
    }
    else {
        D = (D-this.aspects[2])
    }
    
    this.aspects[3] = (L+R+U+D) / 4;
    
    this.aspects[4] = this.links.length / this.aspects[2]
}

function Link(x, y) {
    this.x = x
    this.y = y
}