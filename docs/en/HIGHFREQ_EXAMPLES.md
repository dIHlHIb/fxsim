# Runnable examples for high-frequency blocks

Save any JSON block below as `request.json` and run `python3 -I engine/fxgraph < request.json` from the package root. Successful simulations report the output values and types. The final three requests illustrate configurations that return `UNSUPPORTED`.

## Discrete Filter, Direct form II, every internal type inherited

```json
{
 "design": {
  "blocks": {
   "B": {
    "Numerator": "[0.0675 0.135 0.0675]",
    "Denominator": "[1 -1.143 0.4128]",
    "FilterStructure": "Direct form II",
    "type": "DiscreteFilter"
   },
   "y": {
    "type": "Outport"
   },
   "x": {
    "type": "Inport"
   }
  },
  "lines": [
   {
    "src": "x",
    "dst": "B"
   },
   {
    "src": "B",
    "dst": "y"
   }
  ]
 },
 "inputs": {
  "x": {
   "s": 1,
   "wl": 18,
   "fl": 9,
   "ints": [
    -131072,
    131071,
    0,
    1,
    -1,
    120446,
    -53733,
    48760,
    -9164,
    25284,
    -39035,
    80901,
    -54846,
    79528,
    64061,
    -45339
   ]
  }
 }
}
```

## Discrete Filter, Direct form I, internal types inherited, output written

```json
{
 "design": {
  "blocks": {
   "B": {
    "Numerator": "[0.0675 0.135 0.0675]",
    "Denominator": "[1 -1.143 0.4128]",
    "OutDataTypeStr": "fixdt(1,16,12)",
    "InitialStates": "0",
    "FilterStructure": "Direct form I",
    "type": "DiscreteFilter"
   },
   "y": {
    "type": "Outport"
   },
   "x": {
    "type": "Inport"
   }
  },
  "lines": [
   {
    "src": "x",
    "dst": "B"
   },
   {
    "src": "B",
    "dst": "y"
   }
  ]
 },
 "inputs": {
  "x": {
   "s": 1,
   "wl": 10,
   "fl": 8,
   "ints": [
    153,
    -343,
    147,
    -23,
    -281,
    -328,
    390,
    -295,
    458,
    -160,
    94,
    136,
    -351,
    271,
    104,
    274
   ]
  }
 }
}
```

## Discrete Transfer Fcn, internal types inherited, output written

```json
{
 "design": {
  "blocks": {
   "B": {
    "Numerator": "[0.2929 0.5858 0.2929]",
    "Denominator": "[1 0 0.1716]",
    "OutDataTypeStr": "fixdt(1,24,16)",
    "InitialStates": "0",
    "type": "DiscreteTransferFcn"
   },
   "y": {
    "type": "Outport"
   },
   "x": {
    "type": "Inport"
   }
  },
  "lines": [
   {
    "src": "x",
    "dst": "B"
   },
   {
    "src": "B",
    "dst": "y"
   }
  ]
 },
 "inputs": {
  "x": {
   "s": 1,
   "wl": 12,
   "fl": 10,
   "ints": [
    1148,
    976,
    -666,
    -603,
    1713,
    1866,
    -210,
    788,
    1006,
    -1207,
    -524,
    -569,
    -1937,
    -827,
    1737,
    1017
   ]
  }
 }
}
```

## Biquad Filter, one section, output written

```json
{
 "design": {
  "blocks": {
   "B": {
    "BiQuadCoeffs": "[0.0675 0.135 0.0675 1 -1.143 0.4128]",
    "outputMode": "Binary point scaling",
    "outputWordLength": "32",
    "outputFracLength": "24",
    "type": "BiquadFilter"
   },
   "y": {
    "type": "Outport"
   },
   "x": {
    "type": "Inport"
   }
  },
  "lines": [
   {
    "src": "x",
    "dst": "B"
   },
   {
    "src": "B",
    "dst": "y"
   }
  ]
 },
 "inputs": {
  "x": {
   "s": 1,
   "wl": 24,
   "fl": 12,
   "ints": [
    -8388608,
    8388607,
    0,
    1,
    -1,
    8315526,
    2860103,
    -865018,
    1106622,
    -5666996,
    -3736236,
    -6022064,
    -1236933,
    -6438897,
    -3832336,
    1713267
   ]
  }
 }
}
```

## Discrete FIR 'Lattice MA', explicit types

```json
{
 "design": {
  "blocks": {
   "B": {
    "FilterStructure": "Lattice MA",
    "Coefficients": "[-0.2357 -0.7555 -0.7953 -0.8528 0.1561 -0.5429 0.7601 -0.4704]",
    "CoefDataTypeStr": "fixdt(1,16,15)",
    "ProductDataTypeStr": "fixdt(1,32,28)",
    "AccumDataTypeStr": "fixdt(1,36,28)",
    "OutDataTypeStr": "fixdt(1,24,16)",
    "type": "DiscreteFir"
   },
   "y": {
    "type": "Outport"
   },
   "x": {
    "type": "Inport"
   }
  },
  "lines": [
   {
    "src": "x",
    "dst": "B"
   },
   {
    "src": "B",
    "dst": "y"
   }
  ]
 },
 "inputs": {
  "x": {
   "s": 1,
   "wl": 16,
   "fl": 14,
   "ints": [
    -32768,
    32767,
    0,
    1,
    -1,
    5593,
    -16050,
    -9856,
    11835,
    16820,
    -8116,
    9809,
    20060,
    26815,
    -16183,
    -26154
   ]
  }
 }
}
```

## Discrete FIR 'Lattice MA', library types, small coefficients

```json
{
 "design": {
  "blocks": {
   "B": {
    "FilterStructure": "Lattice MA",
    "Coefficients": "[0.2 -0.15 0.05]",
    "type": "DiscreteFir"
   },
   "y": {
    "type": "Outport"
   },
   "x": {
    "type": "Inport"
   }
  },
  "lines": [
   {
    "src": "x",
    "dst": "B"
   },
   {
    "src": "B",
    "dst": "y"
   }
  ]
 },
 "inputs": {
  "x": {
   "s": 1,
   "wl": 16,
   "fl": 8,
   "ints": [
    -32768,
    32767,
    0,
    1,
    -1,
    -21753,
    8441,
    -21741,
    2520,
    -30611,
    16475,
    21639,
    -28992,
    32388,
    14464,
    -4166
   ]
  }
 }
}
```

## HDL NCO, LUTCompress on, 4-bit dither

```json
{
 "design": {
  "blocks": {
   "B": {
    "PhaseIncrementSource": "Property",
    "PhaseIncrement": "2083",
    "AccumulatorWL": "24",
    "NumQuantizerAccumulatorBits": "12",
    "Waveform": "Sine",
    "DitherSource": "Property",
    "OutputWL": "16",
    "OutputFL": "10",
    "LUTCompress": "on",
    "NumDitherBits": "4",
    "type": "HDLNCO"
   },
   "y1": {
    "type": "Outport"
   },
   "y2": {
    "type": "Outport"
   },
   "v": {
    "type": "Inport"
   }
  },
  "lines": [
   {
    "src": "v",
    "dst": "B"
   },
   {
    "src": "B",
    "dst": "y1"
   },
   {
    "src": "B",
    "sport": 2,
    "dst": "y2"
   }
  ]
 },
 "inputs": {
  "v": {
   "type": "boolean",
   "ints": [
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1
   ]
  }
 }
}
```

## HDL Counter with a reset port, CountSampTime 2

```json
{
 "design": {
  "blocks": {
   "B": {
    "CountType": "Count limited",
    "CountWordLen": "4",
    "CountMax": "9",
    "CountSampTime": "2",
    "CountResetPort": "on",
    "type": "HDLCounter"
   },
   "y": {
    "type": "Outport"
   },
   "rst": {
    "type": "Inport",
    "SampleTime": "2"
   }
  },
  "lines": [
   {
    "src": "rst",
    "dst": "B"
   },
   {
    "src": "B",
    "dst": "y"
   }
  ]
 },
 "inputs": {
  "rst": {
   "type": "boolean",
   "ints": [
    1,
    0,
    0,
    0,
    0,
    0,
    1,
    0,
    0,
    0,
    1,
    0,
    0,
    0,
    0,
    0
   ]
  }
 }
}
```

## HDL Counter, CountSampTime 3, no ports

```json
{
 "design": {
  "blocks": {
   "B": {
    "CountType": "Modulo",
    "CountWordLen": "8",
    "CountMax": "9",
    "CountSampTime": "3",
    "type": "HDLCounter"
   },
   "y": {
    "type": "Outport"
   }
  },
  "lines": [
   {
    "src": "B",
    "dst": "y"
   }
  ]
 },
 "inputs": {},
 "nsteps": 16
}
```

## Refusals

### Lattice MA after a fast-to-slow Rate Transition (multirate: not recorded)

```json
{
 "design": {
  "blocks": {
   "u": {
    "type": "Inport",
    "SampleTime": "1"
   },
   "R": {
    "type": "RateTransition",
    "OutPortSampleTime": "2"
   },
   "F": {
    "type": "DiscreteFir",
    "FilterStructure": "Lattice MA",
    "Coefficients": "[0.5 0.25]"
   },
   "y": {
    "type": "Outport"
   }
  },
  "lines": [
   {
    "src": "u",
    "dst": "R"
   },
   {
    "src": "R",
    "dst": "F"
   },
   {
    "src": "F",
    "dst": "y"
   }
  ]
 },
 "inputs": {
  "u": {
   "s": 1,
   "wl": 16,
   "fl": 8,
   "ints": [
    256,
    -300,
    100,
    50,
    -20,
    10,
    5,
    7
   ]
  }
 },
 "stop_time": "7"
}
```

### HDL NCO, LUTCompress on with a 14-bit quantiser (not recorded)

```json
{
 "design": {
  "blocks": {
   "v": {
    "type": "Inport"
   },
   "N": {
    "type": "HDLNCO",
    "PhaseIncrementSource": "Property",
    "PhaseIncrement": "517",
    "DitherSource": "None",
    "LUTCompress": "on",
    "NumQuantizerAccumulatorBits": "14"
   },
   "y": {
    "type": "Outport"
   },
   "v_out": {
    "type": "Outport"
   }
  },
  "lines": [
   {
    "src": "v",
    "dst": "N"
   },
   {
    "src": "N",
    "dst": "y"
   },
   {
    "src": "N",
    "sport": 2,
    "dst": "v_out"
   }
  ]
 },
 "inputs": {
  "v": {
   "type": "boolean",
   "ints": [
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1
   ]
  }
 }
}
```

### HDL Counter with a reset port at CountSampTime 2, its input at the base step (not recorded)

```json
{
 "design": {
  "blocks": {
   "r": {
    "type": "Inport"
   },
   "C": {
    "type": "HDLCounter",
    "CountType": "Count limited",
    "CountWordLen": "4",
    "CountMax": "9",
    "CountSampTime": "2",
    "CountResetPort": "on"
   },
   "y": {
    "type": "Outport"
   }
  },
  "lines": [
   {
    "src": "r",
    "dst": "C"
   },
   {
    "src": "C",
    "dst": "y"
   }
  ]
 },
 "inputs": {
  "r": {
   "type": "boolean",
   "ints": [
    0,
    0,
    1,
    0,
    0,
    0
   ]
  }
 }
}
```
