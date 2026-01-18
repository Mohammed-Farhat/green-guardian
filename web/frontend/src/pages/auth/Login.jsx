import { useState } from 'react'
import { useNavigate } from 'react-router-dom'
import './login.css'

export default function Login() {
  const [email, setEmail] = useState('')
  const [password, setPassword] = useState('')
  const [error, setError] = useState('')
  const navigate = useNavigate()

  function handleSubmit(e) {
    e.preventDefault()
    if (!email || !password) {
      setError('Please enter email and password')
      return
    }
    // Simple mock auth: store a token in localStorage
    console.log('Logging in with', { email, password })
    localStorage.setItem('gg_token', 'demo-token')
    navigate('/robots')
  }

  return (
    <div className="login-page">
      <form className="login-card" onSubmit={handleSubmit}>
        <h2>Sign In</h2>
        {error && <div className="error">{error}</div>}
        <label>
          Email
          <input type="email" value={email} onChange={e => setEmail(e.target.value)} />
        </label>
        <label>
          Password
          <input type="password" value={password} onChange={e => setPassword(e.target.value)} />
        </label>
        <button type="submit">Login</button>
      </form>
    </div>
  )
}
