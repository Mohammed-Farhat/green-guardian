import { useEffect, useState } from 'react'
import { BrowserRouter, Routes, Route, Navigate } from 'react-router-dom'
import './App.css'
import Login from './pages/auth/Login'
import RobotSelect from './pages/robots/RobotSelect'
import Dashboard from './pages/dashboard/Dashboard'
import { ToastContainer } from 'react-toastify'
import 'react-toastify/dist/ReactToastify.css'
function AppRouter() {
  const [isAuth, setIsAuth] = useState(false)

  useEffect(() => {
    const token = localStorage.getItem('gg_token')
    setIsAuth(!!token)
  }, [])

  return (
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
        <Route path="/login" element={<Login />} />
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
  )
}

export default AppRouter
