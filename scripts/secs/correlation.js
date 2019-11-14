(() => {
  const
    empty = '',
    space = ' ',
    precision = 2,
    highPrecision = 4,
    color_base = 0xeeeeee,
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
    calc_average = arr => (arr.reduce((sum, curr) => sum + curr, 0) / arr.length),
    calc_standard_deviation = arr => {
      const arr_avg = calc_average(arr)
      const sum_d = (sum, curr) => sum + Math.pow(curr - arr_avg, 2)
      return Math.sqrt(arr.reduce(sum_d, 0) / arr.length)
    },
    // en: use it as a action at every element
    calc_standard_unit = arr => {
      const arr_sd = calc_standard_deviation(arr)
      const arr_avg = calc_average(arr)
      return (
        0 === arr_sd ?
          arr.map(() => 1) : // here 0 and 1 mean same
          arr.map(num => (num - arr_avg) / arr_sd)
      )
    },
    calc_correlation_for_lists = lists => {
      const [x_arr, y_arr] = lists
      const limit = Math.min(x_arr.length, y_arr.length)
      const x_arr_in_su = calc_standard_unit(x_arr)
      const y_arr_in_su = calc_standard_unit(y_arr)
      let sum = 0
      for (let i = 0; i < limit; ++i) {
        sum += (x_arr_in_su[i] * y_arr_in_su[i])
      }
      return (sum / limit).toPrecision(precision)
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

    newLine(lineValues) {
      const
        limit = lineValues.length,
        line = new_svg('polyline'),
        increment = this.#width / (limit - 1),
        xCoords = [],
        lists = [xCoords, lineValues]
      for (let i = 0, x = 0; i < limit; ++i, x += increment) {
        xCoords.push(x)
      }

      line.svg_points = lists2pairs(lists).map(([x, y]) => `${x},${y}`).join(space)
      line.svg_stroke = random_color()
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
    const
      svgCanvas = new SC(),
      outputText = document.createElement('span'),
      firstPoint = svgCanvas.newPoint(20, 20),
      generateRandomValues = () => {
        const
          limit = 260,
          valueDomain = 300,
          result = []

        for (let x = 0; x < limit; ++x) {
          result.push(Math.ceil(Math.random() * valueDomain))
        }
        return result
      },
      lineAValues = generateRandomValues(),
      lineBValues = generateRandomValues(),
      toCalc = [lineAValues, lineBValues]

    document.body.appendChild(outputText)
    outputText.innerHTML = `cr of pairs to draw: ${calc_correlation_for_lists(toCalc)}`
    svgCanvas
      .addChild(svgCanvas.newPoint(50, 50))
      .addChild(firstPoint)
      .cutChild(firstPoint)
      .addChild(svgCanvas.newLine(lineAValues))
      .addChild(svgCanvas.newLine(lineBValues))
  }
})()
