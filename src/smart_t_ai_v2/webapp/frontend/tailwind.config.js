/** @type {import('tailwindcss').Config} */
export default {
  content: ['./index.html', './src/**/*.{js,jsx,ts,tsx}'],
  theme: {
    extend: {
      colors: {
        trolley: {
          green:  '#22c55e',
          dark:   '#0f172a',
          card:   '#1e293b',
          border: '#334155',
        },
      },
    },
  },
  plugins: [],
}
