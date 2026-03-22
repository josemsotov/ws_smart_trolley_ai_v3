// Service Worker — SmartTrolley PWA
const CACHE = 'smarttrolley-v1'
const PRECACHE = ['/', '/index.html']

self.addEventListener('install', e =>
  e.waitUntil(
    caches.open(CACHE).then(c => c.addAll(PRECACHE))
  )
)

self.addEventListener('fetch', e => {
  // Solo cachear peticiones GET de la misma origin
  if (e.request.method !== 'GET') return
  if (e.request.url.includes('/video_feed')) return   // nunca cachear MJPEG
  if (e.request.url.includes('/ws')) return           // nunca cachear WS

  e.respondWith(
    fetch(e.request).catch(() => caches.match(e.request))
  )
})
