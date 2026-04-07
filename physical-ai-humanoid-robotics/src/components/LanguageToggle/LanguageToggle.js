// src/components/LanguageToggle/LanguageToggle.js
import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

function LanguageToggle() {
  const [currentLanguage, setCurrentLanguage] = useState('english');
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
    // Check for saved language preference in localStorage
    const savedLanguage = localStorage.getItem('preferredLanguage');
    if (savedLanguage && (savedLanguage === 'english' || savedLanguage === 'urdu')) {
      setCurrentLanguage(savedLanguage);
    }
  }, []);

  const toggleLanguage = () => {
    const newLanguage = currentLanguage === 'english' ? 'urdu' : 'english';
    setCurrentLanguage(newLanguage);
    // Save preference to localStorage
    localStorage.setItem('preferredLanguage', newLanguage);
  };

  // Don't render on the server to avoid hydration mismatches
  if (!isClient) {
    return (
      <div className={styles.languageTogglePlaceholder}>
        <button
          className={styles.languageToggleButton}
          disabled
        >
          Loading...
        </button>
      </div>
    );
  }

  return (
    <div className={styles.languageToggleContainer}>
      <button
        className={styles.languageToggleButton}
        onClick={toggleLanguage}
        aria-label={`Switch to ${currentLanguage === 'english' ? 'Urdu' : 'English'} language`}
      >
        {currentLanguage === 'english' ? 'Translate into Urdu' : 'Reset into English'}
      </button>

      {/* Styles to hide/show content based on language preference */}
      <style jsx>{`
        .urdu-content { display: ${currentLanguage === 'urdu' ? 'block' : 'none'} !important; }
        .english-content { display: ${currentLanguage === 'english' ? 'block' : 'none'} !important; }
      `}</style>
    </div>
  );
}

export default LanguageToggle;
