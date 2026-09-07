/** @type {import('tailwindcss').Config} */
export default {
    content: [
        "./index.html",
        "./src/**/*.{js,ts,jsx,tsx}",
    ],
    theme: {
        extend: {
            colors: {
                'ika-dark': '#0f172a',
                'ika-accent': '#3b82f6',
                'ika-danger': '#ef4444',
                'ika-success': '#22c55e',
            },
        },
    },
    plugins: [],
}
