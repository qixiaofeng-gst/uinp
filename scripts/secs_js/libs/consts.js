const {
  set_fn_as_key
} = require('./obj_utils')

const symbol_types = {
  ad: {
    desc: 'ADR',
    explain: 'American depositary receipt'
  },
  re: {
    desc: 'REIT',
    explain: 'real estate investment trust'
  },
  ce: {
    desc: 'Closed end fund'
  },
  si: {
    desc: 'Secondary Issue'
  },
  lp: {
    desc: 'Limited PartnerShips'
  },
  cs: {
    desc: 'Common stock'
  },
  et: {
    desc: 'ETF',
    explain: 'Exchange-traded fund'
  }
}
set_fn_as_key(symbol_types)

module.exports = {
  symbol_types
}
