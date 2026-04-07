import React, { useState, useRef, useEffect } from 'react';
import NavbarItem from '@theme-original/NavbarItem';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { useSession, signOut } from '@site/src/lib/auth-client';

function AuthButtons() {
  // Use Better Auth React hook - no provider needed
  const { data: session, isPending } = useSession();
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const dropdownRef = useRef(null);

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target)) {
        setIsDropdownOpen(false);
      }
    };

    if (isDropdownOpen) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isDropdownOpen]);

  const handleSignOut = async () => {
    try {
      await signOut();
      setIsDropdownOpen(false);
      window.location.href = '/';
    } catch (error) {
      console.error('Sign out error:', error);
    }
  };

  // Capitalize first letter of name
  const capitalizeFirstLetter = (str) => {
    if (!str) return '';
    return str.charAt(0).toUpperCase() + str.slice(1);
  };

  // Don't show anything while loading
  if (isPending) {
    return null;
  }

  // User is logged in - Show dropdown
  if (session?.user) {
    const displayName = capitalizeFirstLetter(session.user.name || session.user.email?.split('@')[0] || 'User');
    const userEmail = session.user.email;

    return (
      <div style={{ position: 'relative', marginLeft: '12px' }} ref={dropdownRef}>
        {/* Dropdown Trigger Button */}
        <button
          onClick={() => setIsDropdownOpen(!isDropdownOpen)}
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: '8px',
            padding: '6px 12px',
            background: 'var(--ifm-color-primary)',
            color: 'white',
            border: 'none',
            borderRadius: '8px',
            cursor: 'pointer',
            fontSize: '14px',
            fontWeight: '500',
            transition: 'all 0.2s',
          }}
          onMouseOver={(e) => {
            e.currentTarget.style.background = 'var(--ifm-color-primary-dark)';
          }}
          onMouseOut={(e) => {
            e.currentTarget.style.background = 'var(--ifm-color-primary)';
          }}
        >
          {/* User Avatar Circle */}
          <div
            style={{
              width: '24px',
              height: '24px',
              borderRadius: '50%',
              background: 'rgba(255, 255, 255, 0.2)',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              fontSize: '12px',
              fontWeight: 'bold',
            }}
          >
            {displayName.charAt(0).toUpperCase()}
          </div>
          <span>{displayName}</span>
          {/* Dropdown Arrow */}
          <svg
            width="12"
            height="12"
            viewBox="0 0 12 12"
            fill="none"
            style={{
              transform: isDropdownOpen ? 'rotate(180deg)' : 'rotate(0deg)',
              transition: 'transform 0.2s',
            }}
          >
            <path
              d="M2 4L6 8L10 4"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
            />
          </svg>
        </button>

        {/* Dropdown Menu */}
        {isDropdownOpen && (
          <div
            style={{
              position: 'absolute',
              top: 'calc(100% + 8px)',
              right: '0',
              minWidth: '220px',
              background: 'var(--ifm-background-surface-color)',
              border: '1px solid var(--ifm-color-emphasis-300)',
              borderRadius: '8px',
              boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
              zIndex: 1000,
              overflow: 'hidden',
              animation: 'dropdownFadeIn 0.2s ease-out',
            }}
          >
            {/* User Info Section */}
            <div
              style={{
                padding: '12px 16px',
                borderBottom: '1px solid var(--ifm-color-emphasis-200)',
              }}
            >
              <div
                style={{
                  fontSize: '14px',
                  fontWeight: '600',
                  color: 'var(--ifm-font-color-base)',
                  marginBottom: '4px',
                }}
              >
                {displayName}
              </div>
              <div
                style={{
                  fontSize: '12px',
                  color: 'var(--ifm-color-emphasis-600)',
                  wordBreak: 'break-all',
                }}
              >
                {userEmail}
              </div>
            </div>

            {/* Sign Out Button */}
            <button
              onClick={handleSignOut}
              style={{
                width: '100%',
                padding: '10px 16px',
                background: 'transparent',
                border: 'none',
                textAlign: 'left',
                cursor: 'pointer',
                fontSize: '14px',
                color: 'var(--ifm-color-danger)',
                fontWeight: '500',
                transition: 'background 0.2s',
              }}
              onMouseOver={(e) => {
                e.currentTarget.style.background = 'var(--ifm-color-emphasis-100)';
              }}
              onMouseOut={(e) => {
                e.currentTarget.style.background = 'transparent';
              }}
            >
              Sign Out
            </button>
          </div>
        )}

        {/* CSS Animation */}
        <style>{`
          @keyframes dropdownFadeIn {
            from {
              opacity: 0;
              transform: translateY(-8px);
            }
            to {
              opacity: 1;
              transform: translateY(0);
            }
          }
        `}</style>
      </div>
    );
  }

  // User is not logged in
  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: '8px', marginLeft: '12px' }}>
      <a
        href="/signin"
        style={{
          padding: '6px 16px',
          borderRadius: '6px',
          fontSize: '14px',
          fontWeight: '500',
          textDecoration: 'none',
          color: 'var(--ifm-navbar-link-color)',
          border: '1px solid var(--ifm-navbar-link-color)',
          background: 'transparent',
          transition: 'all 0.2s'
        }}
        onMouseOver={(e) => {
          e.currentTarget.style.color = 'var(--ifm-color-primary)';
          e.currentTarget.style.borderColor = 'var(--ifm-color-primary)';
        }}
        onMouseOut={(e) => {
          e.currentTarget.style.color = 'var(--ifm-navbar-link-color)';
          e.currentTarget.style.borderColor = 'var(--ifm-navbar-link-color)';
        }}
      >
        Sign In
      </a>
      <a
        href="/signup"
        style={{
          padding: '6px 16px',
          background: 'var(--ifm-color-primary)',
          color: 'white',
          border: '1px solid var(--ifm-color-primary)',
          borderRadius: '6px',
          fontSize: '14px',
          fontWeight: '500',
          textDecoration: 'none',
          transition: 'all 0.2s'
        }}
        onMouseOver={(e) => {
          e.currentTarget.style.background = 'var(--ifm-color-primary-dark)';
          e.currentTarget.style.borderColor = 'var(--ifm-color-primary-dark)';
          e.currentTarget.style.transform = 'translateY(-1px)';
        }}
        onMouseOut={(e) => {
          e.currentTarget.style.background = 'var(--ifm-color-primary)';
          e.currentTarget.style.borderColor = 'var(--ifm-color-primary)';
          e.currentTarget.style.transform = 'translateY(0)';
        }}
      >
        Sign Up
      </a>
    </div>
  );
}

export default function NavbarItemWrapper(props) {
  // Render original navbar item
  const item = <NavbarItem {...props} />;

  // Add auth buttons after the last navbar item (GitHub link)
  if (props.label === 'GitHub') {
    return (
      <>
        {item}
        <BrowserOnly fallback={<div />}>
          {() => <AuthButtons />}
        </BrowserOnly>
      </>
    );
  }

  return item;
}
