// Copyright 2006 The Closure Library Authors. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS-IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
goog.provide('goog.color.alphaTest');
goog.setTestOnly('goog.color.alphaTest');

goog.require('goog.array');
goog.require('goog.color');
goog.require('goog.color.alpha');
goog.require('goog.testing.jsunit');

function testIsValidAlphaHexColor() {
  var goodAlphaHexColors = ['#ffffffff', '#ff781259', '#01234567', '#Ff003DaB',
                            '#3CAF', '#abcdefab', '#3CAB'];
  var badAlphaHexColors = ['#xxxxxxxx', '88990077', 'not_color', '#123456789',
                           'fffffgfg'];
  for (var i = 0; i < goodAlphaHexColors.length; i++) {
    assertTrue(goodAlphaHexColors[i],
        goog.color.alpha.isValidAlphaHexColor_(goodAlphaHexColors[i]));
  }
  for (var i = 0; i < badAlphaHexColors.length; i++) {
    assertFalse(badAlphaHexColors[i],
        goog.color.alpha.isValidAlphaHexColor_(badAlphaHexColors[i]));
  }
}

function testIsValidRgbaColor() {
  var goodRgbaColors = ['rgba(255, 0, 0, 1)', 'rgba(255,127,0,1)',
                        'rgba(0,0,255,0.5)', '(255, 26, 75, 0.2)',
                        'RGBA(0, 55, 0, 0.6)', 'rgba(0, 200, 0, 0.123456789)'];
  var badRgbaColors = ['(255, 0, 0)', '(2555,0,0, 0)', '(1,2,3,4,5)',
                       'rgba(1,20,)', 'RGBA(20,20,20,)', 'RGBA',
                       'rgba(255, 0, 0, 1.1)'];
  for (var i = 0; i < goodRgbaColors.length; i++) {
    assertEquals(goodRgbaColors[i], 4,
                 goog.color.alpha.isValidRgbaColor_(goodRgbaColors[i]).length);
  }
  for (var i = 0; i < badRgbaColors.length; i++) {
    assertEquals(badRgbaColors[i], 0,
                 goog.color.alpha.isValidRgbaColor_(badRgbaColors[i]).length);
  }
}

function testIsValidHslaColor() {
  var goodHslaColors = ['hsla(120, 0%, 0%, 1)', 'hsla(360,20%,0%,1)',
                        'hsla(0,0%,50%,0.5)', 'HSLA(0, 55%, 0%, 0.6)',
                        'hsla(0, 85%, 0%, 0.123456789)'];
  var badHslaColors = ['(255, 0, 0, 0)', 'hsla(2555,0,0, 0)', 'hsla(1,2,3,4,5)',
                       'hsla(1,20,)', 'HSLA(20,20,20,)',
                       'hsla(255, 0, 0, 1.1)', 'HSLA'];
  for (var i = 0; i < goodHslaColors.length; i++) {
    assertEquals(goodHslaColors[i], 4,
                 goog.color.alpha.isValidHslaColor_(goodHslaColors[i]).length);
  }
  for (var i = 0; i < badHslaColors.length; i++) {
    assertEquals(badHslaColors[i], 0,
                 goog.color.alpha.isValidHslaColor_(badHslaColors[i]).length);
  }
}

function testParse() {
  var colors = ['rgba(15, 250, 77, 0.5)', '(127, 127, 127, 0.8)', '#ffeeddaa',
                '12345678', 'hsla(160, 50%, 90%, 0.2)'];
  var parsed = goog.array.map(colors, goog.color.alpha.parse);
  assertEquals('rgba', parsed[0].type);
  assertEquals(goog.color.alpha.rgbaToHex(15, 250, 77, 0.5), parsed[0].hex);
  assertEquals('rgba', parsed[1].type);
  assertEquals(goog.color.alpha.rgbaToHex(127, 127, 127, 0.8), parsed[1].hex);
  assertEquals('hex', parsed[2].type);
  assertEquals('#ffeeddaa', parsed[2].hex);
  assertEquals('hex', parsed[3].type);
  assertEquals('#12345678', parsed[3].hex);
  assertEquals('hsla', parsed[4].type);
  assertEquals('#d9f2ea33', parsed[4].hex);

  var badColors = ['rgb(01, 1, 23)', '(256, 256, 256)', '#ffeeddaa'];
  for (var i = 0; i < badColors.length; i++) {
    var e = assertThrows(badColors[i] + ' is not a valid color string',
        goog.partial(goog.color.parse, badColors[i]));
    assertContains('Error processing ' + badColors[i],
        'is not a valid color string', e.message);
  }
}

function testHexToRgba() {
  var testColors = [['#B0FF2D66', [176, 255, 45, 0.4]],
                    ['#b26e5fcc', [178, 110, 95, 0.8]],
                    ['#66f3', [102, 102, 255, 0.2]]];

  for (var i = 0; i < testColors.length; i++) {
    var r = goog.color.alpha.hexToRgba(testColors[i][0]);
    var t = testColors[i][1];

    assertEquals('Red channel should match.', t[0], r[0]);
    assertEquals('Green channel should match.', t[1], r[1]);
    assertEquals('Blue channel should match.', t[2], r[2]);
    assertEquals('Alpha channel should match.', t[3], r[3]);
  }

  var badColors = ['', '#g00', 'some words'];
  for (var i = 0; i < badColors.length; i++) {
    var e = assertThrows(
        goog.partial(goog.color.alpha.hexToRgba, badColors[i]));
    assertEquals("'" + badColors[i] + "' is not a valid alpha hex color",
        e.message);
  }
}

function testHexToRgbaStyle() {
  assertEquals('rgba(255,0,0,1)',
               goog.color.alpha.hexToRgbaStyle('#ff0000ff'));
  assertEquals('rgba(206,206,206,0.8)',
               goog.color.alpha.hexToRgbaStyle('#cecececc'));
  assertEquals('rgba(51,204,170,0.2)',
               goog.color.alpha.hexToRgbaStyle('#3CA3'));
  assertEquals('rgba(1,2,3,0.016)',
               goog.color.alpha.hexToRgbaStyle('#01020304'));
  assertEquals('rgba(255,255,0,0.333)',
               goog.color.alpha.hexToRgbaStyle('#FFFF0055'));

  var badHexColors = ['#12345', null, undefined, '#.1234567890'];
  for (var i = 0; i < badHexColors.length; ++i) {
    var e = assertThrows(badHexColors[i] + ' is an invalid hex color',
        goog.partial(goog.color.alpha.hexToRgbaStyle, badHexColors[i]));
    assertEquals("'" + badHexColors[i] + "' is not a valid alpha hex color",
        e.message);
  }
}

function testRgbaToHex() {
  assertEquals('#af13ffff', goog.color.alpha.rgbaToHex(175, 19, 255, 1));
  assertEquals('#357cf099', goog.color.alpha.rgbaToHex(53, 124, 240, 0.6));
  var badRgba = [[-1, -1, -1, -1], [0, 0, 0, 2], ['a', 'b', 'c', 'd'],
                 [undefined, 5, 5, 5]];
  for (var i = 0; i < badRgba.length; ++i) {
    var e = assertThrows(badRgba[i] + ' is not a valid rgba color',
        goog.partial(goog.color.alpha.rgbaArrayToHex, badRgba[i]));
    assertContains('is not a valid RGBA color', e.message);
  }
}

function testRgbaToRgbaStyle() {
  var testColors = [[[175, 19, 255, 1], 'rgba(175,19,255,1)'],
                    [[53, 124, 240, .6], 'rgba(53,124,240,0.6)'],
                    [[10, 20, 30, .1234567], 'rgba(10,20,30,0.123)'],
                    [[20, 30, 40, 1 / 3], 'rgba(20,30,40,0.333)']];

  for (var i = 0; i < testColors.length; ++i) {
    var r = goog.color.alpha.rgbaToRgbaStyle(testColors[i][0][0],
                                             testColors[i][0][1],
                                             testColors[i][0][2],
                                             testColors[i][0][3]);
    assertEquals(testColors[i][1], r);
  }

  var badColors = [[0, 0, 0, 2]];
  for (var i = 0; i < badColors.length; ++i) {
    var e = assertThrows(goog.partial(goog.color.alpha.rgbaToRgbaStyle,
        badColors[i][0], badColors[i][1], badColors[i][2], badColors[i][3]));

    assertContains('is not a valid RGBA color', e.message);
  }

  // Loop through all bad color values and ensure they fail in each channel.
  var badValues = [-1, 300, 'a', undefined, null, NaN];
  var color = [0, 0, 0, 0];
  for (var i = 0; i < badValues.length; ++i) {
    for (var channel = 0; channel < color.length; ++channel) {
      color[channel] = badValues[i];
      var e = assertThrows(color + ' is not a valid rgba color',
          goog.partial(goog.color.alpha.rgbaToRgbaStyle, color));
      assertContains('is not a valid RGBA color', e.message);

      color[channel] = 0;
    }
  }
}

function testRgbaArrayToRgbaStyle() {
  var testColors = [[[175, 19, 255, 1], 'rgba(175,19,255,1)'],
                    [[53, 124, 240, .6], 'rgba(53,124,240,0.6)']];

  for (var i = 0; i < testColors.length; ++i) {
    var r = goog.color.alpha.rgbaArrayToRgbaStyle(testColors[i][0]);
    assertEquals(testColors[i][1], r);
  }

  var badColors = [[0, 0, 0, 2]];
  for (var i = 0; i < badColors.length; ++i) {
    var e = assertThrows(goog.partial(goog.color.alpha.rgbaArrayToRgbaStyle,
        badColors[i]));

    assertContains('is not a valid RGBA color', e.message);
  }

  // Loop through all bad color values and ensure they fail in each channel.
  var badValues = [-1, 300, 'a', undefined, null, NaN];
  var color = [0, 0, 0, 0];
  for (var i = 0; i < badValues.length; ++i) {
    for (var channel = 0; channel < color.length; ++channel) {
      color[channel] = badValues[i];
      var e = assertThrows(color + ' is not a valid rgba color',
          goog.partial(goog.color.alpha.rgbaToRgbaStyle, color));
      assertContains('is not a valid RGBA color', e.message);

      color[channel] = 0;
    }
  }
}

function testRgbaArrayToHsla() {
  var opaqueBlueRgb = [0, 0, 255, 1];
  var opaqueBlueHsl = goog.color.alpha.rgbaArrayToHsla(opaqueBlueRgb);
  assertArrayEquals('Conversion from RGBA to HSLA should be as expected',
                    [240, 1, 0.5, 1], opaqueBlueHsl);

  var nearlyOpaqueYellowRgb = [255, 190, 0, 0.7];
  var nearlyOpaqueYellowHsl =
      goog.color.alpha.rgbaArrayToHsla(nearlyOpaqueYellowRgb);
  assertArrayEquals('Conversion from RGBA to HSLA should be as expected',
                    [45, 1, 0.5, 0.7], nearlyOpaqueYellowHsl);

  var transparentPurpleRgb = [180, 0, 255, 0];
  var transparentPurpleHsl =
      goog.color.alpha.rgbaArrayToHsla(transparentPurpleRgb);
  assertArrayEquals('Conversion from RGBA to HSLA should be as expected',
                    [282, 1, 0.5, 0], transparentPurpleHsl);
}

function testNormalizeAlphaHex() {
  var compactColor = '#abcd';
  var normalizedCompactColor =
      goog.color.alpha.normalizeAlphaHex_(compactColor);
  assertEquals('The color should have been normalized to the right length',
               '#aabbccdd', normalizedCompactColor);

  var uppercaseColor = '#ABCDEF01';
  var normalizedUppercaseColor =
      goog.color.alpha.normalizeAlphaHex_(uppercaseColor);
  assertEquals('The color should have been normalized to lowercase',
               '#abcdef01', normalizedUppercaseColor);
}

function testHsvaArrayToHex() {
  var opaqueSkyBlueHsv = [190, 1, 255, 1];
  var opaqueSkyBlueHex = goog.color.alpha.hsvaArrayToHex(opaqueSkyBlueHsv);
  assertEquals('The HSVA array should have been properly converted to hex',
               '#00d4ffff', opaqueSkyBlueHex);

  var halfTransparentPinkHsv = [300, 1, 255, 0.5];
  var halfTransparentPinkHex =
      goog.color.alpha.hsvaArrayToHex(halfTransparentPinkHsv);
  assertEquals('The HSVA array should have been properly converted to hex',
               '#ff00ff7f', halfTransparentPinkHex);

  var transparentDarkTurquoiseHsv = [175, 1, 127, 0.5];
  var transparentDarkTurquoiseHex =
      goog.color.alpha.hsvaArrayToHex(transparentDarkTurquoiseHsv);
  assertEquals('The HSVA array should have been properly converted to hex',
               '#007f747f', transparentDarkTurquoiseHex);
}

function testExtractHexColor() {
  var opaqueRed = '#ff0000ff';
  var red = goog.color.alpha.extractHexColor(opaqueRed);
  assertEquals('The hex part of the color should have been extracted correctly',
               '#ff0000', red);

  var halfOpaqueDarkGreenCompact = '#0507';
  var darkGreen =
      goog.color.alpha.extractHexColor(halfOpaqueDarkGreenCompact);
  assertEquals('The hex part of the color should have been extracted correctly',
               '#005500', darkGreen);

}

function testExtractAlpha() {
  var colors = ['#ff0000ff', '#0507', '#ff000005'];
  var expectedOpacities = ['ff', '77', '05'];

  for (var i = 0; i < colors.length; i++) {
    var opacity = goog.color.alpha.extractAlpha(colors[i]);
    assertEquals('The alpha transparency should have been extracted correctly',
                 expectedOpacities[i], opacity);
  }
}

function testHslaArrayToRgbaStyle() {
  assertEquals('rgba(102,255,102,0.5)',
               goog.color.alpha.hslaArrayToRgbaStyle([120, 100, 70, 0.5]));
  assertEquals('rgba(28,23,23,0.9)',
               goog.color.alpha.hslaArrayToRgbaStyle([0, 10, 10, 0.9]));
}

function testRgbaStyleParsableResult() {
  var testColors = [[175, 19, 255, 1],
                    [53, 124, 240, .6],
                    [20, 30, 40, 0.3333333],
                    [255, 255, 255, 0.7071067811865476]];

  for (var i = 0, testColor; testColor = testColors[i]; i++) {
    var rgbaStyle = goog.color.alpha.rgbaStyle_(testColor);
    var parsedColor = goog.color.alpha.hexToRgba(
        goog.color.alpha.parse(rgbaStyle).hex);
    assertEquals(testColor[0], parsedColor[0]);
    assertEquals(testColor[1], parsedColor[1]);
    assertEquals(testColor[2], parsedColor[2]);
    // Parsing keeps a 1/255 accuracy on the alpha channel.
    assertRoughlyEquals(testColor[3], parsedColor[3], 0.005);
  }
}
