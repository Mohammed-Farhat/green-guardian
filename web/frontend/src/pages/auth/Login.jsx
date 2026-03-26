import { useState } from 'react'
import { useNavigate } from 'react-router-dom'
import { useAuth } from '../../App'
import { authApi } from '../../services/api'
import './login.css'

export default function Login() {
  const [email, setEmail] = useState('')
  const [password, setPassword] = useState('')
  const [error, setError] = useState('')
  const [isRegister, setIsRegister] = useState(false)
  const [name, setName] = useState('')
  const [loading, setLoading] = useState(false)
  const navigate = useNavigate()
  const { login } = useAuth()

  async function handleSubmit(e) {
    e.preventDefault()
    if (!email || !password) {
      setError('Please enter email and password')
      return
    }
    if (isRegister && password.length < 6) {
      setError('Password must be at least 6 characters')
      return
    }
    setError('')
    setLoading(true)

    try {
      let data
      if (isRegister) {
        data = await authApi.register(email, password, name)
      } else {
        data = await authApi.login(email, password)
      }
      login(data.token, data.user)
      navigate('/robots')
    } catch (err) {
      setError(err.message)
    } finally {
      setLoading(false)
    }
  }

  return (
    <div className="login-page">
      <form className="login-card" onSubmit={handleSubmit}>
        <h2>{isRegister ? 'Create Account' : 'Sign In'}</h2>
        {error && <div className="error">{error}</div>}
        {isRegister && (
          <label htmlFor="name-input">
            Name
            <input id="name-input" type="text" value={name} onChange={e => setName(e.target.value)} />
          </label>
        )}
        <label htmlFor="email-input">
          Email
          <input id="email-input" type="email" value={email} onChange={e => setEmail(e.target.value)} />
        </label>
        <label htmlFor="password-input">
          Password
          <input id="password-input" type="password" value={password} onChange={e => setPassword(e.target.value)} />
        </label>
        <button type="submit" disabled={loading}>
          {loading ? 'Please wait...' : isRegister ? 'Register' : 'Login'}
        </button>
        <p style={{ marginTop: '12px', textAlign: 'center', fontSize: '14px' }}>
          {isRegister ? 'Already have an account?' : "Don't have an account?"}{' '}
          <button
            type="button"
            style={{
              background: 'none',
              border: 'none',
              color: '#4caf50',
              cursor: 'pointer',
              fontWeight: 600,
              fontSize: '14px',
              padding: 0,
            }}
            onClick={() => { setIsRegister(!isRegister); setError('') }}
          >
            {isRegister ? 'Sign In' : 'Register'}
          </button>
        </p>
      </form>
    </div>
  )
}
