import { createContext, useContext, useState } from 'react'
import { BrowserRouter, Routes, Route, Navigate } from 'react-router-dom'
import './App.css'
import Login from './pages/auth/Login'
import RobotSelect from './pages/robots/RobotSelect'
import Dashboard from './pages/dashboard/Dashboard'
import { ToastContainer } from 'react-toastify'
import 'react-toastify/dist/ReactToastify.css'

const AuthContext = createContext(null)

export function useAuth() {
  return useContext(AuthContext)
}

function AppRouter() {
  const [isAuth, setIsAuth] = useState(!!localStorage.getItem('gg_token'))

  function login(token, user) {
    localStorage.setItem('gg_token', token)
    localStorage.setItem('gg_user', JSON.stringify(user))
    setIsAuth(true)
  }

  function logout() {
    localStorage.removeItem('gg_token')
    localStorage.removeItem('gg_user')
    localStorage.removeItem('selectedRobot')
    setIsAuth(false)
  }

  return (
    <AuthContext.Provider value={{ isAuth, login, logout }}>
      <BrowserRouter>
        <ToastContainer
          position="top-right"
          autoClose={6000}
          hideProgressBar={false}
          newestOnTop
          closeOnClick
          pauseOnHover
          draggable
        />
        <Routes>
          <Route path="/login" element={isAuth ? <Navigate to="/robots" replace /> : <Login />} />
          <Route
            path="/robots"
            element={isAuth ? <RobotSelect /> : <Navigate to="/login" replace />}
          />
          <Route
            path="/dashboard"
            element={isAuth ? <Dashboard /> : <Navigate to="/login" replace />}
          />
          <Route path="/" element={<Navigate to={isAuth ? '/robots' : '/login'} replace />} />
          <Route path="*" element={<Navigate to={isAuth ? '/robots' : '/login'} replace />} />
        </Routes>
      </BrowserRouter>
    </AuthContext.Provider>
  )
}

export default AppRouter
