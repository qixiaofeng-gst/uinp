(() => {
  const
    empty = '',
    space = ' ',
    precision = 2,
    highPrecision = 4,
    color_base = 0xeeeeee,
    // running_days_per_year = 260,
    clamp_float = num => parseFloat(num.toFixed(highPrecision)),
    clamp_str = (str, width = 6) => str.length > width ? str.substr(0, width) : str + `${(() => {
      const limit = width - str.length
      let result = empty
      for (let i = 0; i < limit; ++i) {
        result += '0'
      }
      return result
    })()}`,
    random_color = () => `#${clamp_str(Math.ceil(Math.random() * color_base).toString(16))}`,
    lists2pairs = lists => {
      const
        [listA, listB] = lists,
        pairs = []
      const limit = Math.min(listA.length, listB.length)
      for (let i = 0; i < limit; ++i) {
        pairs.push([listA[i], listB[i]])
      }
      return pairs
    },
    xmlns = 'http://www.w3.org/2000/svg',
    msg_never_touch_me = 'Never touch me',
    never_touch_me = new Proxy({}, {
      get(_target, _p, _receiver) {
        throw msg_never_touch_me
      },
      set(_target, _p, _value, _receiver) {
        return false
      },
    }),
    add = Symbol('add'),
    cut = Symbol('cut'),
    feed = Symbol('feed'),
    name = Symbol('name'),
    view = Symbol('view'),
    new_svg = tag => {
      const
        element = document.createElementNS(xmlns, tag),
        svg_prefix = 'svg_',
        css_prefix = 'css_',
        _2dash = str => str.replace(/_/g, '-'),
        accessors = {
          get: (_target, p, _receiver) => {
            switch (p) {
              case name: {
                return tag
              }
              case view: {
                return element
              }
              case add: {
                return child => element.appendChild(child)
              }
              case cut: {
                return child => element.removeChild(child)
              }
              case feed: {
                return () => {
                  throw 'Not implemented'
                }
              }
              default:
                console.error(`${p} is touched unexpectedly.`)
                throw msg_never_touch_me
            }
          },
          set: (_target, p, value, _receiver) => {
            if (p.startsWith(svg_prefix)) {
              element.setAttributeNS(null, _2dash(p.substr(4)), value)
            } else if (p.startsWith(css_prefix)) {
              element.style[p.substr(4)] = value
            } else {
              element[p] = value
            }
            return true
          },
        }

      return new Proxy(never_touch_me, accessors)
    }

  class NiceArray {
    #array = []
    #count = 0
    #max = Number.MIN_SAFE_INTEGER
    #min = Number.MAX_SAFE_INTEGER
    #sum = 0
    #averages = []
    #deviationSquares = []
    #standardDeviations = []
    #standardUnits = []

    set newNumber(number) {
      this.#count++
      this.#array.push(number)
      this.#sum += number
      this.#averages.push(clamp_float(this.#sum / this.#count))
      if (this.#max < number) {
        this.#max = number
      }
      if (this.#min > number) {
        this.#min = number
      }
      this.#deviationSquares.push(0)
      this.#standardUnits.push(0)
      this.#calcStandardDeviation()
      this.#calcStandardUnits()
    }

    get standardDeviation() {
      return this.#standardDeviations[this.#count - 1]
    }

    get average() {
      return this.#averages[this.#count - 1]
    }

    get averages() {
      return this.#averages
    }

    get array() {
      return this.#array
    }

    constructor() {
    }

    addNumbers(numbers) {
      numbers.forEach(number => this.newNumber = number)
      return this
    }

    calcCorrelationTo(niceArray) {
      this.#debug(false)
      niceArray.#debug(false)
      const isNiceArray = niceArray instanceof NiceArray
      if (false === isNiceArray) {
        return 0
      }
      const limit = Math.min(niceArray.#count, this.#count)
      let sum = 0
      for (let i = 0; i < limit; ++i) {
        sum += niceArray.#standardUnits[i] * this.#standardUnits[i]
      }
      return (sum / limit).toFixed(precision)
    }

    #debug = function (isEnable = true) {
      if (false === isEnable) {
        return
      }
      const self = this
      console.log(`======= debug output start`)
      console.log(`total elements: ${self.#count}`)
      console.log(`1. deviation squares: ${self.#deviationSquares}`)
      console.log(`2. standard deviations: ${self.#standardDeviations}`)
      console.log(`3. standard units: ${self.#standardUnits}`)
      console.log(`4. averages: ${self.#averages}`)
      console.log(`5. array elements: ${self.#array}`)
      console.log(`======= debug output end`)
    }

    #calcStandardDeviation = function () {
      const self = this
      const limit = self.#count
      const average = self.average
      for (let i = 0; i < limit; ++i) {
        const deviation = self.#array[i] - average
        self.#deviationSquares[i] = clamp_float(deviation * deviation)
      }
      const deviationSquareAverage = self.#deviationSquares.reduce((sum, current) => sum + current, 0) / limit
      self.#standardDeviations.push(clamp_float(Math.sqrt(deviationSquareAverage)))
    }

    #calcStandardUnits = function () {
      const
        self = this,
        limit = self.#count,
        standardDeviation = self.standardDeviation,
        average = self.average

      if (0 === standardDeviation) {
        self.#standardUnits[limit - 1] = 1
      } else {
        for (let i = 0; i < limit; ++i) {
          self.#standardUnits[i] = clamp_float((self.#array[i] - average) / standardDeviation)
        }
      }
    }
  }

  class SC {
    #svgProxy = new_svg('svg')
    #width = 800
    #height = 300
    #niceLines = []

    constructor() {
      this.#svgProxy.css_width = `${this.#width}px`
      this.#svgProxy.css_height = `${this.#height}px`
      this.#svgProxy.svg_viewBox = `0 0 ${this.#width} ${this.#height}`
      document.body.appendChild(this.#svgProxy[view])
    }

    addChild(elementProxy) {
      this.#svgProxy[add](elementProxy[view])
      return this
    }

    cutChild(elementProxy) {
      this.#svgProxy[cut](elementProxy[view])
      return this
    }

    newPoint(x, y) {
      const circle = new_svg('circle')
      circle.svg_cx = x
      circle.svg_cy = y
      circle.svg_r = 1
      return circle
    }

    newNiceLine(numbers) {
      const
        index = this.#niceLines.length,
        niceArray = new NiceArray().addNumbers(numbers),
        niceLine = {
          model: niceArray,
          color: random_color(),
          valueLine: false,
          averageLine: false,
          standardDeviationLine: false,
          standardUnitLine: false,
          deviationSquareLine: false,
        },
        valueLine = this.#newLine(niceArray.array)
      valueLine.svg_stroke = niceLine.color
      this.addChild(valueLine)
      this.#niceLines.push(niceLine)
      return index
    }

    addNumberToNiceLine(number, iLine) {
      // TODO Just do it
      console.log('Here add number and update')
      return this
    }

    activateAverageLineFor(iLine, isActivate = true) {
      const niceLine = this.#niceLines[iLine]
      if (false === niceLine.averageLine) {
        niceLine.averageLine = this.#newLine(niceLine.model.averages)
        niceLine.averageLine.svg_stroke = niceLine.color
        niceLine.averageLine.css_display = 'none'
        this.addChild(niceLine.averageLine)
      }
      if (isActivate) {
        niceLine.averageLine.css_display = 'unset'
      } else {
        niceLine.averageLine.css_display = 'none'
      }
      return this
    }

    calcCorrelationOf(iLineA, iLineB) {
      const
        niceLineA = this.#niceLines[iLineA],
        niceLineB = this.#niceLines[iLineB]
      return niceLineA.model.calcCorrelationTo(niceLineB.model)
    }

    #newLine = function (lineValues) {
      const limit = lineValues.length
      if (limit < 2) {
        throw 'At least two values should be given.'
      }
      const
        self = this,
        line = new_svg('polyline'),
        increment = self.#width / (limit - 1),
        xCoords = [],
        lists = [xCoords, lineValues]
      for (let i = 0, x = 0; i < limit; ++i, x += increment) {
        xCoords.push(x)
      }

      line.svg_points = lists2pairs(lists).map(([x, y]) => `${x},${y}`).join(space)
      line.svg_stroke = 'black'
      line.svg_stroke_width = 1
      line.svg_fill_opacity = 0
      return line
    }
  }

  console.log(
    new NiceArray().addNumbers(
      [1, 1, 1, 1, 1],
    ).calcCorrelationTo(new NiceArray().addNumbers(
      [.2, .3, .6, .8, 30],
    )), 'expected: 0.00')
  console.log(
    new NiceArray().addNumbers(
      [1, 2, 3, 4, 6],
    ).calcCorrelationTo(new NiceArray().addNumbers(
      [.2, .3, .6, .8, 30],
    )), 'expected: 0.82')
  console.log(
    new NiceArray().addNumbers(
      [1, 1, 1, 1, 1],
    ).calcCorrelationTo(new NiceArray().addNumbers(
      [.1, .1, .1, .1, .1],
    )), 'expected: 1.00')

  window.onload = () => {
    let addedValueCount = 2
    const
      valueCountLimit = 50,
      svgCanvas = new SC(),
      outputText = document.createElement('span'),
      firstPoint = svgCanvas.newPoint(20, 20),
      generateRandomValues = () => {
        const
          valueDomain = 300,
          result = []

        for (let x = 0; x < valueCountLimit; ++x) {
          result.push(Math.ceil(Math.random() * valueDomain))
        }
        return result
      },
      lineAValues = generateRandomValues(),
      lineBValues = generateRandomValues(),
      iLineA = svgCanvas.newNiceLine(lineAValues.slice(0, 2)),
      iLineB = svgCanvas.newNiceLine(lineBValues.slice(0, 2)),
      timeInterval = 1000,
      animate = (startTime = 0) => {
        const doIt = deltaTime => {
          const
            timePassed = deltaTime - startTime,
            shouldUpdate = timeInterval < timePassed,
            timeForNext = shouldUpdate ? deltaTime : startTime
          if (shouldUpdate) {
            svgCanvas
              .addNumberToNiceLine(lineAValues[addedValueCount], iLineA)
              .addNumberToNiceLine(lineAValues[addedValueCount], iLineB)
            addedValueCount++
          }
          if (addedValueCount < valueCountLimit) {
            animate(timeForNext)
          } else {
            console.log('Animation end.')
          }
        }
        requestAnimationFrame(doIt)
      }

    outputText.innerHTML = `cr of pairs to draw: ${svgCanvas.calcCorrelationOf(iLineA, iLineB)}`
    document.body.appendChild(outputText)
    svgCanvas
      .addChild(firstPoint)
      .cutChild(firstPoint)
      .activateAverageLineFor(iLineA)
    animate()
  }
})()
