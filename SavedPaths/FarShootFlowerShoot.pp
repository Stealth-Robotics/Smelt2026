{
  "startPoint": {
    "x": 60,
    "y": 132,
    "locked": false,
    "headingDeg": 90
  },
  "lines": [
    {
      "id": "line-mvq73vio-m2muie",
      "color": "#ffc516",
      "name": "goShoot",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 60,
        "y": 121
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "startDeg": 90,
        "endDeg": 88
      }
    },
    {
      "kind": "atomic",
      "id": "line-mvq7pb0m-fw4i4q",
      "endPoint": {
        "x": 47.5,
        "y": 120
      },
      "controlPoints": [],
      "heading": {
        "type": "constant",
        "reverse": false,
        "degrees": 90
      },
      "color": "#C7767A",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "name": "afterFirstShot"
    },
    {
      "kind": "atomic",
      "id": "line-mvq7s9o5-dv7ra8",
      "endPoint": {
        "x": 47.5,
        "y": 132
      },
      "controlPoints": [],
      "heading": {
        "type": "constant",
        "reverse": true,
        "degrees": 90
      },
      "color": "#AD7AB6",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "name": "collectFromFlower"
    },
    {
      "kind": "atomic",
      "id": "line-mvq83hy4-pc3oih",
      "endPoint": {
        "x": 49,
        "y": 116
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "reverse": true,
        "startDeg": 90,
        "endDeg": 109
      },
      "color": "#DAC9BD",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "name": "shootAgain"
    },
    {
      "kind": "atomic",
      "id": "line-mvq8bdct-wfa0mo",
      "endPoint": {
        "x": 15.0343228200371,
        "y": 123.9935064935065
      },
      "controlPoints": [
        {
          "x": 40.79406307977736,
          "y": 124.65491651205936
        }
      ],
      "heading": {
        "type": "tangential",
        "reverse": false
      },
      "color": "#C7DB75",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "name": "park"
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 141.5,
          "y": 70
        },
        {
          "x": 141.5,
          "y": 141.5
        },
        {
          "x": 118.3,
          "y": 141.5
        },
        {
          "x": 135.5,
          "y": 118
        },
        {
          "x": 136.3,
          "y": 70.2
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6.2,
          "y": 116.9
        },
        {
          "x": 25,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 6,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-mvq73vio-m2muie"
    },
    {
      "kind": "path",
      "lineId": "line-mvq7pb0m-fw4i4q"
    },
    {
      "kind": "path",
      "lineId": "line-mvq7s9o5-dv7ra8"
    },
    {
      "kind": "path",
      "lineId": "line-mvq83hy4-pc3oih"
    },
    {
      "kind": "path",
      "lineId": "line-mvq8bdct-wfa0mo"
    }
  ],
  "fieldPoints": [],
  "activePaths": [],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 17,
    "rHeight": 18,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "biobuzz.webp",
    "robotImage": "/robot.png",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false,
    "showHeadingArrow": false,
    "showCurrentTValue": false,
    "leftPanelWidth": 370,
    "rightPanelWidth": 620,
    "headingArrowLength": 50,
    "headingArrowColor": "#ffffff",
    "headingArrowThickness": 2,
    "pathOpacity": 1,
    "leftPanelMinWidth": 0,
    "rightPanelMinWidth": 0,
    "penToolMaxPaths": 8,
    "curveThroughMaxPoints": 4,
    "experimentalFeatures": {
      "optimize": false,
      "curveThrough": false
    }
  },
  "version": "1.5.0",
  "timestamp": "2026-10-27T05:16:10.292Z"
}