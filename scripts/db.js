/** Database Infrastructure. (Based on mongodb) */

const { MongoClient:db, ObjectID:OID } = require('mongodb')

let client_instance = false
let db_instance = false
let is_preparing = false

const url = 'mongodb://localhost:27017'
const db_name = 'be-album'
const cached_callbacks = []

const exec_callbacks = (cb) => {
  const temp = cached_callbacks.splice(0, cached_callbacks.length)
  for (let item of temp) {
    item()
  }
  cb && cb()
}

const connect = cb => {
  if (is_preparing) {
    cached_callbacks.push(cb)
    return
  }
  if (client_instance) {
    exec_callbacks(cb)
    return
  }
  is_preparing = true
  const ts = Date.now()
  db.connect(url, { useNewUrlParser: true }).then(client => {
    is_preparing = false
    console.log(`Connect cost ${Date.now() - ts}ms.`)
    client_instance = client
    db_instance = client.db(db_name)
    exec_callbacks(cb)
  }).catch(err => {
    is_preparing = false
    console.log('Failed to connect. Error:', err)
  })
}

module.exports = collection_name => {

  const insert = (obj, cb) => connect(() => {
    const ts = Date.now()
    db_instance.collection(collection_name).insertOne(obj).then(({ result, ops: [{ _id }] }) => {
      cb && cb({ result, _id })
      console.log(`Insert operation on ${collection_name} cost ${Date.now() - ts}ms.`)
    }).catch(err => {
      console.log('Failed to insert:', obj, 'Error:', err)
    })
  })

  const update = (filter, obj, cb) => connect(() => {
    const ts = Date.now()
    db_instance.collection(collection_name).updateMany(filter, {
      $set: obj
    }).then(({ result }) => {
      cb && cb(result)
      console.log(`UpdateWith operation on ${collection_name} cost ${Date.now() - ts}ms.`)
    }).catch(err => {
      console.log('Failed to update:', obj, 'Error:', err)
    })
  })

  const list = (filter, cb) => connect(() => {
    const ts = Date.now()
    db_instance.collection(collection_name).find(filter).toArray().then(arr => {
      cb && cb(arr)
      console.log(`List operation on ${collection_name} cost ${Date.now() - ts}ms.`)
    }).catch(err => {
      console.log('Failed to list. Error:', err)
    })
  })

  const get = (query, cb) => connect(() => {
    const ts = Date.now()
    db_instance.collection(collection_name).findOne(query).then(entry => {
      cb && cb(entry)
      console.log(`Get operation on ${collection_name} cost ${Date.now() - ts}ms.`)
    }).catch(err => {
      console.log('Failed to get. Error:', err)
    })
  })

	const exec = cb => connect(() => {
		const ts = Date.now()
    cb && cb(
			db_instance.collection(collection_name),
			() => console.log(`Free querying on ${collection_name} cost ${Date.now() - ts}ms.`)
		)
  })

  return ({
    insert, list, get, update, exec
  })
}
