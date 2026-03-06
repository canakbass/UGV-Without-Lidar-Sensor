import React from 'react';

const Button = ({ children, variant = 'primary', className = '', ...props }) => {
    const baseStyles = "px-4 py-2 rounded font-bold transition-all active:scale-95 disabled:opacity-50 disabled:cursor-not-allowed";

    const variants = {
        primary: "bg-ika-accent hover:bg-blue-500 text-white shadow-lg shadow-blue-900/20",
        secondary: "bg-slate-700 hover:bg-slate-600 text-white",
        danger: "bg-ika-danger hover:bg-red-500 text-white shadow-lg shadow-red-900/20",
        success: "bg-ika-success hover:bg-green-500 text-white shadow-lg shadow-green-900/20",
        outline: "border-2 border-slate-600 text-slate-300 hover:border-slate-500 hover:text-white"
    };

    return (
        <button
            className={`${baseStyles} ${variants[variant]} ${className}`}
            {...props}
        >
            {children}
        </button>
    );
};

export default Button;
