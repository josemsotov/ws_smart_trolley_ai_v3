/**
 * useWebSocket — hook compartido para comunicación con el backend Pi5
 *
 * Uso:
 *   const { connected, lastMessage, send } = useWebSocket()
 */
import { useState, useEffect, useRef, useCallback } from 'react'

// Resolución automática de la URL del servidor (mismo host, puerto 8000)
const WS_URL = `ws://${window.location.hostname}:8000/ws`

export default function useWebSocket() {
  const [connected, setConnected]     = useState(false)
  const [lastMessage, setLastMessage] = useState(null)
  const wsRef   = useRef(null)
  const retryRef = useRef(null)

  const connect = useCallback(() => {
    if (wsRef.current?.readyState === WebSocket.OPEN) return

    const ws = new WebSocket(WS_URL)
    wsRef.current = ws

    ws.onopen = () => {
      setConnected(true)
      clearTimeout(retryRef.current)
    }

    ws.onmessage = (e) => {
      try {
        setLastMessage(JSON.parse(e.data))
      } catch {/* ignorar */ }
    }

    ws.onclose = () => {
      setConnected(false)
      // Reconexión automática cada 3 s
      retryRef.current = setTimeout(connect, 3000)
    }

    ws.onerror = () => ws.close()
  }, [])

  useEffect(() => {
    connect()
    return () => {
      clearTimeout(retryRef.current)
      wsRef.current?.close()
    }
  }, [connect])

  const send = useCallback((action, data = {}) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ action, data }))
    }
  }, [])

  return { connected, lastMessage, send }
}
